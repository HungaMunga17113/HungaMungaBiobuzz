package org.firstinspires.ftc.teamcode.helpers;

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
    },
    SIGMOID {
        @Override public double apply(double x, CurveParams p) {
            double a = p.sharpness;
            if (a == 0) return x;
            double t = Math.abs(x);
            double sig  = 1.0 / (1.0 + Math.exp(-a * t));
            double sig0 = 0.5;
            double sig1 = 1.0 / (1.0 + Math.exp(-a));
            return Math.signum(x) * (sig - sig0) / (sig1 - sig0);
        }
    };

    public abstract double apply(double x, CurveParams p);
}
