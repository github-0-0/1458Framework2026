package frc.robot.lib.util;

public record PIDConstants(double p, double i, double d, double iZone) {
    public double kP() {
        return p;
    }

    public double kI() {
        return p;
    }

    public double kD() {
        return p;
    }

    public double iZone() {
        return iZone;
    }
}
