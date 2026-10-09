package org.firstinspires.ftc.teamcode.pedro.mcl;

import com.qualcomm.robotcore.hardware.AnalogInput;

import java.util.ArrayList;
import java.util.List;
import java.util.Random;

/**
 * fancy math stuff that only aiden and owen understand
 * only the particle filter part for this class; pedro things are in MCLLocalizer
 * reads the distance sensors once / step
 * only gives x/y estimates
 */
public class MCL {
    private static final double MM_PER_INCH = 25.4;
    public static class RangeSensor {
        final AnalogInput sensor;
        final double forwardOffset;
        final double lateralOffset;
        final double turn;
        final double maxRangeMm;

        public RangeSensor(AnalogInput sensor, double forwardOffset, double lateralOffset, double turn, double maxRangeMm) {
            this.sensor = sensor;
            this.forwardOffset = forwardOffset;
            this.lateralOffset = lateralOffset;
            this.turn = turn;
            this.maxRangeMm = maxRangeMm;
        }

        double readInches() {
            return sensor.getVoltage() / sensor.getMaxVoltage() * maxRangeMm / MM_PER_INCH;
        }
    }

    private static class Particle {
        double x, y, weight;

        Particle(double x, double y, double weight) {
            this.x = x;
            this.y = y;
            this.weight = weight;
        }

        Particle copy() {
            return new Particle(x, y, weight);
        }
    }

    private static class Reading {
        final double reading, forwardOffset, lateralOffset, turn;

        Reading(double reading, double forwardOffset, double lateralOffset, double turn) {
            this.reading = reading;
            this.forwardOffset = forwardOffset;
            this.lateralOffset = lateralOffset;
            this.turn = turn;
        }
    }

    private final List<RangeSensor> sensors;
    /** polyline of field walls/obstacles, {x, y} in inches (pedro coords) */
    private final double[][] fieldMap;
    private final int numParticles;
    private final double sigma;
    /** raycast cap, inches (was 78.75) */
    private final double maxReading;
    /** expected distances beyond this are ignored, inches (was 54) */
    private final double maxUsableReading;
    private final Random gen = new Random();

    private List<Particle> particles = new ArrayList<>();
    private double estimateX, estimateY;

    public MCL(List<RangeSensor> sensors, double[][] fieldMap, int numParticles, double sigma,
               double maxReading, double maxUsableReading) {
        this.sensors = sensors;
        this.fieldMap = fieldMap;
        this.numParticles = numParticles;
        this.sigma = sigma;
        this.maxReading = maxReading;
        this.maxUsableReading = maxUsableReading;
    }

    /** reseed particles gaussian around (x, y) */
    public void reset(double x, double y, double spread) {
        particles = new ArrayList<>(numParticles);
        for (int i = 0; i < numParticles; i++) {
            particles.add(new Particle(x + gen.nextGaussian() * spread, y + gen.nextGaussian() * spread, 1.0 / numParticles));
        }
        estimateX = x;
        estimateY = y;
    }

    public double getEstimateX() {
        return estimateX;
    }

    public double getEstimateY() {
        return estimateY;
    }

    public void mclStep(double deltaX, double deltaY, double theta) {
        List<Reading> frame = new ArrayList<>();
        for (RangeSensor sensor : sensors) {
            frame.add(new Reading(sensor.readInches(), sensor.forwardOffset, sensor.lateralOffset, sensor.turn));
        }

        // no sensors -> nothing to correct with, just follow odom
        if (frame.isEmpty()) {
            for (Particle p : particles) {
                p.x += deltaX;
                p.y += deltaY;
            }
            estimateX += deltaX;
            estimateY += deltaY;
            return;
        }

        double invTwoSigmaSq = 1.0 / (2.0 * sigma * sigma);
        double cos = Math.cos(theta);
        double sin = Math.sin(theta);

        for (Particle p : particles) {
            p.weight = 1.0;
            // move each particle by odom deltas along with noise
            p.x += deltaX + gen.nextGaussian() * sigma;
            p.y += deltaY + gen.nextGaussian() * sigma;

            // simulate readings for each particle and then compare them to actual readings
            // weight each particle (1/sigma*sqrt(2*pi)*e^-((reading - actual)^2/(2*sigma^2)))
            for (Reading read : frame) {
                // sensor position, accounting for its offset on the robot
                double sx = p.x + read.forwardOffset * cos - read.lateralOffset * sin;
                double sy = p.y + read.forwardOffset * sin + read.lateralOffset * cos;

                double minDistance = maxReading;
                double localTheta = theta + read.turn;
                double ry = Math.sin(localTheta);
                double rx = Math.cos(localTheta);

                for (int i = 0; i < fieldMap.length - 1; i++) {
                    double x1 = fieldMap[i][0];
                    double y1 = fieldMap[i][1];
                    double x2 = fieldMap[i + 1][0];
                    double y2 = fieldMap[i + 1][1];

                    if (Math.min(x1, x2) > sx + maxReading && rx > 0) {
                        continue;
                    }

                    double denom = (x1 - x2) * ry - (y1 - y2) * rx;
                    if (Math.abs(denom) < 1e-6) {
                        continue;
                    }
                    double t = ((x1 - sx) * ry - (y1 - sy) * rx) / denom;
                    double u = ((x1 - x2) * (y1 - sy) - (y1 - y2) * (x1 - sx)) / denom;

                    if (0 <= t && t <= 1 && u >= 0) {
                        if (u < minDistance) {
                            minDistance = u;
                        }
                    }
                }

                if (minDistance > maxUsableReading) {
                    continue;
                }
                // Weight the particle based on the minimum distance
                double particleError = minDistance - read.reading;
                double weightHold = (1.0 / (minDistance * Math.sqrt(2 * Math.PI) * sigma)) * Math.exp(-1 * (particleError * particleError) * invTwoSigmaSq);
                p.weight *= weightHold;
            }
        }

        // Resample particles based on weight using Stochastic Universal Sampling
        double particleSum = 0;
        for (Particle p : particles) {
            particleSum += p.weight;
        }
        if (!(particleSum > 0) || Double.isInfinite(particleSum)) {
            // every particle got ~0 weight (bad reading), keep them all equally instead of NaN-ing
            for (Particle p : particles) {
                p.weight = 1.0 / numParticles;
            }
        } else {
            for (Particle p : particles) {
                p.weight /= particleSum;
            }
        }
        double step = 1.0 / numParticles;
        double r = gen.nextDouble() * step;
        double c = particles.get(0).weight;
        int i = 0;
        List<Particle> newParticles = new ArrayList<>(numParticles);
        for (int m = 0; m < numParticles; m++) {
            double U = r + m * step;
            while (U > c && i < particles.size() - 1) {
                i++;
                c += particles.get(i).weight;
            }
            newParticles.add(particles.get(i).copy());
        }
        particles = newParticles;

        double meanX = 0, meanY = 0;
        for (Particle p : particles) {
            meanX += p.x;
            meanY += p.y;
        }
        estimateX = meanX / particles.size();
        estimateY = meanY / particles.size();
    }
}
