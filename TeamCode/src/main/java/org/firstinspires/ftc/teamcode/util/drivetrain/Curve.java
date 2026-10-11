package org.firstinspires.ftc.teamcode.util.drivetrain;

public enum Curve {
    LINEAR {
        @Override public double apply(double x, CurveParams p) { return x; }
    },
    CUBIC_BEZIER {
        @Override public double apply(double x, CurveParams p) {
            double t = Math.abs(x), it = 1 - t;
            return Math.signum(x) * (3 * it * it * t * p.bezierP1 + 3 * it * t * t * p.bezierP2 + t * t * t);
        }
    },
    SMOOTHSTEP {
        @Override public double apply(double x, CurveParams p) {
            double t = Math.abs(x);
            return Math.signum(x) * (t * t * (3 - 2 * t));
        }
    },
    EXPONENTIAL {
        @Override public double apply(double x, CurveParams p) {
            double a = p.sharpness;
            if (a == 0) return x;
            double t = Math.abs(x);
            return Math.signum(x) * ((Math.exp(a * t) - 1) / (Math.exp(a) - 1));
        }
    },
    QUINTIC {
        @Override public double apply(double x, CurveParams p) {
            double t = Math.abs(x);
            return Math.signum(x) * (t * t * t * (t * (t * 6 - 15) + 10));
        }
    };

    public abstract double apply(double x, CurveParams p);
}
