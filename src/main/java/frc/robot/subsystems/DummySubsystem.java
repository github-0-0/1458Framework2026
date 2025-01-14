package frc.robot.subsystems;
import frc.robot.Loops.ILooper;
import frc.robot.Loops.Loop;
public class DummySubsystem extends Subsystem {

	private static DummySubsystem m_Instance;

    public static DummySubsystem getInstance() {
		if (m_Instance == null) {
			m_Instance = new DummySubsystem();
		}
		return m_Instance;
	}

    @Override
	public void writeToLog() {

    }

    @Override
	public void readPeriodicInputs() {
        //System.out.println("Example Subsystem is reading periodic inputs");
    }

    @Override
	public void writePeriodicOutputs() {
        //System.out.println("Example Subsystem is writing periodic outputs");
    }

    @Override
	public void outputTelemetry() {
        //System.out.println("Example Subsystem is outputting telemetry");
    }

    @Override
	public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
			@Override
			public void onStart(double timestamp) {
				//System.out.println("DummySubsystem loop has started");
			}

			@Override
			public void onLoop(double timestamp) {
                //System.out.println("on looped");
			}

			@Override
			public void onStop(double timestamp) {
				//System.out.println("DummySubsystem loop has stopped");
            }
		});
    }

    @Override
	public boolean checkSystem() {
		return false;
	}
}
