package frc.robot.actions;

public abstract class HoldableAction implements Action {
    private volatile boolean isDone;

    @Override
    public final void start() {
        isDone = false;
        onStart();
    }

    public void onStart() { }

    @Override
    public abstract void update();

    public final void finish() { isDone = true; }

    @Override
    public final boolean isFinished() { return isDone; }

    @Override
    public void done() {}
}
