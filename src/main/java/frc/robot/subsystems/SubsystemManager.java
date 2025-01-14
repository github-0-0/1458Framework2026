package frc.robot.subsystems;

import frc.robot.Constants;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
import frc.robot.Loops.Looper;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Manages all subsystems, coordinating reset, start, stop, and updates across them.
 */
public class SubsystemManager implements ILooper {
    // Singleton instance
    private static SubsystemManager instance = null;

    // Subsystem management
    private List<Subsystem> subsystems = new ArrayList<>();
    private final List<Loop> loops = new ArrayList<>();

    // Loop timing (for telemetry)
    private double readTime = 0.0;
    private double onLoopTime = 0.0;
    private double writeTime = 0.0;

    // Private constructor (Singleton)
    private SubsystemManager() {}

    public static SubsystemManager getInstance() {
        if (instance == null) {
            instance = new SubsystemManager();
        }
        return instance;
    }

    // Setters and Getters for subsystems
    public void setSubsystems(Subsystem... subsystemsArray) {
        subsystems = Arrays.asList(subsystemsArray);
    }

    public List<Subsystem> getSubsystems() {
        return subsystems;
    }

    // Telemetry output
    public void outputTelemetry() {
        if (Constants.disableExtraTelemetry) {
            return;
        }
        subsystems.forEach(Subsystem::outputTelemetry);
    }

    // Check if all subsystems are functioning
    public boolean checkSubsystems() {
        return subsystems.stream().allMatch(Subsystem::checkSystem);
    }

    // Stop all subsystems
    public void stop() {
        subsystems.forEach(Subsystem::stop);
    }

    // Loop for when the robot is enabled
    private class EnabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {
            loops.forEach(loop -> loop.onStart(timestamp));
        }

        @Override
        public void onLoop(double timestamp) {

            double startTime = Timer.getFPGATimestamp();

            // Read inputs from subsystems
            subsystems.forEach(Subsystem::readPeriodicInputs);
            readTime = Timer.getFPGATimestamp() - startTime;

            // Execute loops
            loops.forEach(loop -> loop.onLoop(timestamp));
            onLoopTime = Timer.getFPGATimestamp() - (startTime + readTime);

            // Write outputs to subsystems
            subsystems.forEach(Subsystem::writePeriodicOutputs);
            writeTime = Timer.getFPGATimestamp() - (startTime + onLoopTime);

            // Output telemetry
            outputTelemetry();
        }

        @Override
        public void onStop(double timestamp) {
            loops.forEach(loop -> loop.onStop(timestamp));
        }
    }

    // Loop for when the robot is disabled
    private class DisabledLoop implements Loop {
        @Override
        public void onStart(double timestamp) {}

        @Override
        public void onLoop(double timestamp) {
            subsystems.forEach(Subsystem::readPeriodicInputs);
            outputTelemetry();
        }

        @Override
        public void onStop(double timestamp) {}
    }

    // Register loops for enabled mode
    public void registerEnabledLoops(Looper enabledLooper) {
        subsystems.forEach(subsystem -> subsystem.registerEnabledLoops(this));
        enabledLooper.register(new EnabledLoop());
    }

    // Register loops for disabled mode
    public void registerDisabledLoops(Looper disabledLooper) {
        disabledLooper.register(new DisabledLoop());
    }

    // Register a loop
    @Override
    public void register(Loop loop) {
        loops.add(loop);
    }

    // Output loop timing telemetry
    public void outputLoopTimes() {
        SmartDashboard.putNumber("LooperTimes/ReadTime", readTime);
        SmartDashboard.putNumber("LooperTimes/OnLoopTime", onLoopTime);
        SmartDashboard.putNumber("LooperTimes/WriteTime", writeTime);
    }
}