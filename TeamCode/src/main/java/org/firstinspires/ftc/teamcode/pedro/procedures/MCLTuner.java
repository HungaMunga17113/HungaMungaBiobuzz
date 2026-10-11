package org.firstinspires.ftc.teamcode.pedro.procedures;

import com.pedropathing.localization.Localizer;
import com.pedropathing.math.Pose;
import com.pedropathing.tuning.autotune.Inputs;
import com.pedropathing.tuning.autotune.Procedure;
import com.pedropathing.tuning.autotune.TuningOpMode;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pedro.mcl.MCL;
import org.firstinspires.ftc.teamcode.pedro.mcl.MCLLocalizer;

import java.util.ArrayList;
import java.util.List;
import java.util.function.Function;

/**
 * tunes MCL filter:
    - sensor full-scale range
    - measurement sigma
    - max usable reading
    - largest particle count within loop
 * outputs Constants.java block when done
 */
public class MCLTuner extends Procedure {
    /** offset in inches, turn in rad */
    public static class SensorSpec {
        public final String name;
        public final double forward, lateral, turn;
        public double maxRangeMm;

        public SensorSpec(String name, double forward, double lateral, double turn, double maxRangeMm) {
            this.name = name;
            this.forward = forward;
            this.lateral = lateral;
            this.turn = turn;
            this.maxRangeMm = maxRangeMm;
        }

        MCL.RangeSensor build(HardwareMap h) {
            return new MCL.RangeSensor(h.get(AnalogInput.class, name), forward, lateral, turn, maxRangeMm);
        }
    }

    private final Function<HardwareMap, Localizer> localizerFunction;
    private final double[][] fieldMap;

    public MCLTuner(Function<HardwareMap, Localizer> localizerFunction, double[][] fieldMap) {
        super("MCL Tuner", "A procedure for tuning the MCL distance sensor algorithm");
        this.localizerFunction = localizerFunction;
        this.fieldMap = fieldMap;
    }

    @Override
    public void run() throws InterruptedException {
        Inputs setup = inputs("Setup", "How many distance sensors go into the particle filter, and how much loop time can the MCL use?");
        Inputs.Field<Integer> sensorCount = setup.i("Number of Distance Sensors").min(1).max(8).withDefault(3);
        Inputs.Field<Boolean> calibrateRange = setup.b("Calibrate Sensor Full-Scale Range").withDefault(true);
        Inputs.Field<Double> loopBudgetMs = setup.d("MCL Loop Time Budget (ms)").min(1.0).max(50.0).withDefault(6.0);
        awaitInputs(setup);

        List<SensorSpec> specs = new ArrayList<>();
        for (int i = 0; i < sensorCount.get(); i++) {
            Inputs page = inputs("Sensor " + (i + 1),
                    "HardwareMap name and mounting of sensor " + (i + 1) + ". "
                            + "Forward is +x on the robot, lateral is +left, turn is "
                            + "facing relative to robot fw in degrees (counterclockwise +)");
            Inputs.Field<String> name = page.s("HardwareMap Name").withoutDefault();
            Inputs.Field<Double> forward = page.d("Forward Offset (in)").withDefault(0.0);
            Inputs.Field<Double> lateral = page.d("Lateral Offset (in)").withDefault(0.0);
            Inputs.Field<Double> turn = page.d("Turn (deg)").min(-360.0).max(360.0).withDefault(0.0);
            Inputs.Field<Double> maxRange = page.d("Full-Scale Range (mm)").min(1.0).withDefault(4000.0);
            awaitInputs(page);

            specs.add(new SensorSpec(name.get(), forward.get(), lateral.get(),
                    Math.toRadians(turn.get()), maxRange.get()));
        }

        runOpMode(new MCLSensorCheck(specs));

        if (calibrateRange.get()) {
            for (SensorSpec spec : specs) {
                Inputs page = inputs(spec.name + " Range Calibration",
                        "Point " + spec.name + " at a flat wall and measure distance from the "
                                + "sensor face to the wall with a tape measure");
                Inputs.Field<Double> actual = page.d("Actual Distance (in)").min(1.0).withDefault(24.0);
                awaitInputs(page);
                spec.maxRangeMm = runOpMode(new MCLRangeScalar(spec, actual.get()));
                result(spec.name + "/maxRangeMm", spec.maxRangeMm);
            }
        }

        // raycast cap
        double maxSensorRange = 0;
        for (SensorSpec spec : specs) {
            maxSensorRange = Math.max(maxSensorRange, spec.maxRangeMm / MCL.MM_PER_INCH);
        }
        double maxReading = Math.min(maxSensorRange, fieldDiagonal());

        Inputs posePage = inputs("Known Pose",
                "Place the robot at a pose measureable accurately, with at least one sensor "
                        + "facing a wall. Heading in degrees, 0 = +x");
        Inputs.Field<Double> poseX = posePage.d("X (in)").withDefault(72.0);
        Inputs.Field<Double> poseY = posePage.d("Y (in)").withDefault(72.0);
        Inputs.Field<Double> poseHeading = posePage.d("Heading (deg)").min(-360.0).max(360.0).withDefault(0.0);
        awaitInputs(posePage);
        Pose known = new Pose(poseX.get(), poseY.get(), Math.toRadians(poseHeading.get()));

        List<Double> noise = runOpMode(new MCLNoise(specs, fieldMap, known, maxReading));
        double sigma = Math.max(noise.get(0), 0.25);
        double worstBias = noise.get(1);

        for (int i = 0; i < specs.size(); i++) {
            result(specs.get(i).name + "/stdDev", noise.get(2 + i));
            result(specs.get(i).name + "/bias", noise.get(2 + specs.size() + i));
        }

        if (worstBias > 3 * sigma) {
            confirmation("Large Measurement Bias",
                    String.format("A sensor is off by %.2f in versus the known pose, well beyond its "
                            + "%.2f in noise. Check offsets, turn, and the field map before trusting "
                            + "the numbers below.", worstBias, sigma));
        }

        double maxUsableReading = runOpMode(new MCLMaxUsable(specs, fieldMap, localizerFunction, known, maxReading, sigma));

        int particles = runOpMode(new MCLParticleBenchmark(specs, fieldMap, maxReading, maxUsableReading, sigma, loopBudgetMs.get()));

        double convergedError = runOpMode(new MCLConvergence(specs, fieldMap, known, particles, sigma, maxReading, maxUsableReading));

        result("mclSigma", sigma);
        result("mclMaxReading", maxReading);
        result("mclMaxUsableReading", maxUsableReading);
        result("mclParticles", particles);
        result("convergedError", convergedError);

        StringBuilder sb = new StringBuilder();
        sb.append("public static int mclParticles = ").append(particles).append(";\n");
        sb.append("public static double mclSigma = ").append(sigma).append(";\n");
        sb.append("public static double mclMaxReading = ").append(maxReading).append(";\n");
        sb.append("public static double mclMaxUsableReading = ").append(maxUsableReading).append(";\n\n");
        sb.append("public static MCL createMCL(HardwareMap h) {\n");
        sb.append("    List<MCL.RangeSensor> sensors = new ArrayList<>();\n");
        for (SensorSpec spec : specs) {
            sb.append(String.format("    addSensor(h, sensors, \"%s\", %s, %s, %s, %s);\n",
                    spec.name, spec.forward, spec.lateral, spec.turn, spec.maxRangeMm));
        }
        sb.append("    return new MCL(sensors, fieldMap, mclParticles, mclSigma, mclMaxReading, mclMaxUsableReading);\n");
        sb.append("}\n\n");
        sb.append("private static void addSensor(HardwareMap h, List<MCL.RangeSensor> sensors, String name,\n");
        sb.append("                              double forward, double lateral, double turn, double maxRangeMm) {\n");
        sb.append("    AnalogInput input = h.tryGet(AnalogInput.class, name);\n");
        sb.append("    if (input != null) {\n");
        sb.append("        sensors.add(new MCL.RangeSensor(input, forward, lateral, turn, maxRangeMm));\n");
        sb.append("    }\n");
        sb.append("}");
        code(Language.JAVA, sb.toString());
    }

    private double fieldDiagonal() {
        double minX = Double.MAX_VALUE, minY = Double.MAX_VALUE;
        double maxX = -Double.MAX_VALUE, maxY = -Double.MAX_VALUE;
        for (double[] point : fieldMap) {
            minX = Math.min(minX, point[0]);
            maxX = Math.max(maxX, point[0]);
            minY = Math.min(minY, point[1]);
            maxY = Math.max(maxY, point[1]);
        }
        return Math.hypot(maxX - minX, maxY - minY);
    }

    static List<MCL.RangeSensor> build(HardwareMap h, List<SensorSpec> specs) {
        List<MCL.RangeSensor> sensors = new ArrayList<>();
        for (SensorSpec spec : specs) {
            sensors.add(spec.build(h));
        }
        return sensors;
    }
}

/** live readings for confirmation */
class MCLSensorCheck extends TuningOpMode<Boolean> {
    private final List<MCLTuner.SensorSpec> specs;

    MCLSensorCheck(List<MCLTuner.SensorSpec> specs) {
        super("Sensor Check",
                "Shows live distances for all configured sensors. Wave your hand in front of each one "
                        + "to confirm correlation, then stop the OpMode",
                true);
        this.specs = specs;
    }

    @Override
    protected Boolean runTuningOpMode() throws InterruptedException {
        List<MCL.RangeSensor> sensors = MCLTuner.build(hardwareMap, specs);
        waitForStart();
        while (!isStopRequested()) {
            for (int i = 0; i < sensors.size(); i++) {
                telemetry.addData(specs.get(i).name, "%.2f in (%.3f full scale)",
                        sensors.get(i).readInches(), sensors.get(i).readRatio());
            }
            telemetry.update();
        }
        return true;
    }
}

/** maxRangeMm = actual * 25.4 / meanRatio */
class MCLRangeScalar extends TuningOpMode<Double> {
    private final MCLTuner.SensorSpec spec;
    private final double actual;

    MCLRangeScalar(MCLTuner.SensorSpec spec, double actual) {
        super("Range Calibration: " + spec.name,
                "Hold the robot still with " + spec.name + " exactly " + actual
                        + " in from a flat wall, then stop the OpMode",
                true);
        this.spec = spec;
        this.actual = actual;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        MCL.RangeSensor sensor = spec.build(hardwareMap);
        waitForStart();

        double sum = 0;
        int samples = 0;
        while (!isStopRequested()) {
            sum += sensor.readRatio();
            samples++;
            telemetry.addData("samples", samples);
            telemetry.addData("mean full scale", samples == 0 ? 0 : sum / samples);
            telemetry.update();
        }

        if (samples == 0) {
            return spec.maxRangeMm;
        }
        double meanRatio = sum / samples;

        if (meanRatio < 1e-4) {
            return spec.maxRangeMm;
        }
        return actual * MCL.MM_PER_INCH / meanRatio;
    }
}

/**
 * sigma and per-sensor bias @ known pose.
 * returns [sigma, worstAbsBias, stdDev per sensor..., bias per sensor...]
 */
class MCLNoise extends TuningOpMode<List<Double>> {
    private final List<MCLTuner.SensorSpec> specs;
    private final double[][] fieldMap;
    private final Pose known;
    private final double maxReading;

    MCLNoise(List<MCLTuner.SensorSpec> specs, double[][] fieldMap, Pose known, double maxReading) {
        super("Noise Identification",
                "Determines the measurement noise (sigma) and each sensor's bias. "
                        + "Leave the robot still at the known pose for a few seconds, then stop the OpMode",
                true);
        this.specs = specs;
        this.fieldMap = fieldMap;
        this.known = known;
        this.maxReading = maxReading;
    }

    @Override
    protected List<Double> runTuningOpMode() throws InterruptedException {
        List<MCL.RangeSensor> sensors = MCLTuner.build(hardwareMap, specs);
        int n = sensors.size();
        double[] sum = new double[n];
        double[] sumSq = new double[n];
        int samples = 0;

        waitForStart();
        while (!isStopRequested()) {
            for (int i = 0; i < n; i++) {
                double reading = sensors.get(i).readInches();
                sum[i] += reading;
                sumSq[i] += reading * reading;
            }
            samples++;
            telemetry.addData("samples", samples);
            telemetry.update();
        }

        List<Double> stdDevs = new ArrayList<>();
        List<Double> biases = new ArrayList<>();
        double worstStdDev = 0, worstBias = 0;
        for (int i = 0; i < n; i++) {
            double mean = samples == 0 ? 0 : sum[i] / samples;
            double variance = samples < 2 ? 0 : Math.max(0, sumSq[i] / samples - mean * mean);
            double stdDev = Math.sqrt(variance);
            double expected = MCL.expectedReading(sensors.get(i), known.x(), known.y(), known.heading(), fieldMap, maxReading);

            double bias = expected >= maxReading - 1e-6 ? 0 : mean - expected;

            stdDevs.add(stdDev);
            biases.add(bias);
            worstStdDev = Math.max(worstStdDev, stdDev);
            worstBias = Math.max(worstBias, Math.abs(bias));
        }

        List<Double> out = new ArrayList<>();
        out.add(worstStdDev);
        out.add(worstBias);
        out.addAll(stdDevs);
        out.addAll(biases);
        return out;
    }
}

/**
 * max distance the readings can be trusted
 * push the robot away from the wall and compare
 * field map's predicted loc for the odometry pose
 */
class MCLMaxUsable extends TuningOpMode<Double> {
    private static final double BUCKET = 6.0;

    private final List<MCLTuner.SensorSpec> specs;
    private final double[][] fieldMap;
    private final Function<HardwareMap, Localizer> localizerFunction;
    private final Pose known;
    private final double maxReading, sigma;

    MCLMaxUsable(List<MCLTuner.SensorSpec> specs, double[][] fieldMap,
                 Function<HardwareMap, Localizer> localizerFunction, Pose known,
                 double maxReading, double sigma) {
        super("Max Usable Reading Identification",
                "Determines the distance past which readings stop being reliable. "
                        + "Starting from the known pose, slowly push the robot away from the wall its facing "
                        + "until the sensors clearly stop tracking correctly, then stop the OpMode",
                true);
        this.specs = specs;
        this.fieldMap = fieldMap;
        this.localizerFunction = localizerFunction;
        this.known = known;
        this.maxReading = maxReading;
        this.sigma = sigma;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        List<MCL.RangeSensor> sensors = MCLTuner.build(hardwareMap, specs);
        Localizer localizer = localizerFunction.apply(hardwareMap);
        localizer.setPose(known);
        localizer.update();
        Thread.sleep(500);

        waitForStart();
        localizer.setPose(known);
        localizer.update();

        int buckets = (int) Math.ceil(maxReading / BUCKET);
        double[] errorSum = new double[buckets];
        int[] errorCount = new int[buckets];

        while (!isStopRequested()) {
            localizer.update();
            Pose pose = localizer.pose();
            if (pose == null || Double.isNaN(pose.x()) || Double.isNaN(pose.y()) || Double.isNaN(pose.heading())) {
                continue;
            }

            for (MCL.RangeSensor sensor : sensors) {
                double expected = MCL.expectedReading(sensor, pose.x(), pose.y(), pose.heading(), fieldMap, maxReading);
                if (expected >= maxReading - 1e-6) {
                    continue;
                }
                int bucket = Math.min(buckets - 1, (int) (expected / BUCKET));
                errorSum[bucket] += Math.abs(sensor.readInches() - expected);
                errorCount[bucket]++;
            }

            telemetry.addData("pose", pose);
            telemetry.update();
        }

        double tolerance = Math.max(3 * sigma, 1.0);
        double usable = 0;
        for (int i = 0; i < buckets; i++) {
            if (errorCount[i] < 5) {
                continue;
            }
            if (errorSum[i] / errorCount[i] > tolerance) {
                break;
            }
            usable = (i + 1) * BUCKET;
        }
        return usable > 0 ? Math.min(usable, maxReading) : maxReading;
    }
}

/** largest particle count for mclStep to fit in loop */
class MCLParticleBenchmark extends TuningOpMode<Integer> {
    private static final int STEP = 50;
    private static final int MAX_PARTICLES = 1500;
    private static final int STEPS_PER_TRIAL = 25;

    private final List<MCLTuner.SensorSpec> specs;
    private final double[][] fieldMap;
    private final double maxReading, maxUsableReading, sigma, budgetMs;

    MCLParticleBenchmark(List<MCLTuner.SensorSpec> specs, double[][] fieldMap, double maxReading,
                         double maxUsableReading, double sigma, double budgetMs) {
        super("Particle Count Identification",
                "Times mclStep at increasing particle counts and picks the largest that fits your "
                        + "loop time. Leave the robot still; will stop on its own",
                false);
        this.specs = specs;
        this.fieldMap = fieldMap;
        this.maxReading = maxReading;
        this.maxUsableReading = maxUsableReading;
        this.sigma = sigma;
        this.budgetMs = budgetMs;
    }

    @Override
    protected Integer runTuningOpMode() throws InterruptedException {
        List<MCL.RangeSensor> sensors = MCLTuner.build(hardwareMap, specs);
        waitForStart();

        int best = STEP;
        ElapsedTime timer = new ElapsedTime();
        for (int count = STEP; count <= MAX_PARTICLES && !isStopRequested(); count += STEP) {
            MCL mcl = new MCL(sensors, fieldMap, count, sigma, maxReading, maxUsableReading);
            mcl.reset(72, 72, MCLLocalizer.START_SPREAD);

            mcl.mclStep(0, 0, 0);

            timer.reset();
            for (int i = 0; i < STEPS_PER_TRIAL; i++) {
                mcl.mclStep(0, 0, 0);
            }
            double msPerStep = timer.milliseconds() / STEPS_PER_TRIAL;

            telemetry.addData("particles", count);
            telemetry.addData("ms/step", "%.2f (budget %.2f)", msPerStep, budgetMs);
            telemetry.addData("best so far", best);
            telemetry.update();

            if (msPerStep > budgetMs) {
                break;
            }
            best = count;
        }
        return best;
    }
}

/** how far off does the filter settle @ known pose with tuned nums */
class MCLConvergence extends TuningOpMode<Double> {
    private static final int SETTLE_STEPS = 50;
    private static final int MEASURE_STEPS = 100;

    private final List<MCLTuner.SensorSpec> specs;
    private final double[][] fieldMap;
    private final Pose known;
    private final int particles;
    private final double sigma, maxReading, maxUsableReading;

    MCLConvergence(List<MCLTuner.SensorSpec> specs, double[][] fieldMap, Pose known, int particles,
                   double sigma, double maxReading, double maxUsableReading) {
        super("Convergence Check",
                "Runs MCL with tuned values while the robot is at known pose and "
                        + "reports steady-state error. Leave the robot still; will stop on its own",
                false);
        this.specs = specs;
        this.fieldMap = fieldMap;
        this.known = known;
        this.particles = particles;
        this.sigma = sigma;
        this.maxReading = maxReading;
        this.maxUsableReading = maxUsableReading;
    }

    @Override
    protected Double runTuningOpMode() throws InterruptedException {
        MCL mcl = new MCL(MCLTuner.build(hardwareMap, specs), fieldMap, particles, sigma, maxReading, maxUsableReading);
        waitForStart();

        mcl.reset(known.x() + 4, known.y() + 4, MCLLocalizer.START_SPREAD);

        for (int i = 0; i < SETTLE_STEPS && !isStopRequested(); i++) {
            mcl.mclStep(0, 0, known.heading());
            telemetry.addData("settling", "%d / %d", i + 1, SETTLE_STEPS);
            telemetry.update();
        }

        double sum = 0;
        int samples = 0;
        for (int i = 0; i < MEASURE_STEPS && !isStopRequested(); i++) {
            mcl.mclStep(0, 0, known.heading());
            sum += Math.hypot(mcl.getEstimateX() - known.x(), mcl.getEstimateY() - known.y());
            samples++;
            telemetry.addData("mean error (in)", "%.2f", sum / samples);
            telemetry.addData("estimate", "%.2f, %.2f", mcl.getEstimateX(), mcl.getEstimateY());
            telemetry.update();
        }

        return samples == 0 ? Double.NaN : sum / samples;
    }
}
