package frc.robot.lib.sysid.actions;

import static edu.wpi.first.units.Units.Volts;

import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.actions.*;
import frc.robot.lib.sysid.subsystems.*;
import frc.robot.lib.util.Stopwatch;

public class MotorQuasistatic implements Action {
    private Stopwatch mStopwatch = null;
    private IMotorSubsystem mMotorSubsystem = null;

    public MotorQuasistatic(IMotorSubsystem motorSubsystem) {
        mMotorSubsystem = motorSubsystem;
        mStopwatch = new Stopwatch();
    }

    @Override
    public void start() {
        mStopwatch.reset();
    }

    @Override
    public void update() {
        mStopwatch.update();
        mMotorSubsystem.setVoltage(
            Volts.of(mStopwatch.getTimeElapsed() / mMotorSubsystem.getRampTime() * mMotorSubsystem.getMaxVoltage().in(Volts))
        );
    }

    @Override
    public void done() {
        mMotorSubsystem.setVoltage(Volts.of(0));
    }

    @Override
    public boolean isFinished() {
        return mStopwatch.getTimeElapsed() > mMotorSubsystem.getRampTime();
    }
}
