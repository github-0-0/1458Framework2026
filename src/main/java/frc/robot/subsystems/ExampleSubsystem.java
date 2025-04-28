package frc.robot.subsystems;
import frc.robot.lib.loops.ILooper;
import frc.robot.lib.loops.Loop;
public class ExampleSubsystem extends Subsystem {
	private static ExampleSubsystem m_Instance;

    public static ExampleSubsystem getInstance() {
		if (m_Instance == null) {
			m_Instance = new ExampleSubsystem();
		}
		return m_Instance;
	}

    @Override
	public void writeToLog() {

    }

    @Override
	public void readPeriodicInputs() {
        //Read inputs here
    }

    @Override
	public void writePeriodicOutputs() {
        //Write outputs here
    }

    @Override
	public void outputTelemetry() {
        //Output telemetry here
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
