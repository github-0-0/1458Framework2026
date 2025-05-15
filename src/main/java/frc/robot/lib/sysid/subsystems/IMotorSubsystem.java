package frc.robot.lib.sysid.subsystems;

import edu.wpi.first.units.measure.Voltage;

public interface IMotorSubsystem {
    public Voltage getMaxVoltage();
    public double getRampTime();
    public double getPosition();
    public double getVelocity();
    public void setVoltage(Voltage voltage);
}
