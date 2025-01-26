package frc.robot.autos.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Repeats an action.
 */
public class RepeatAction implements Action {
	private Action mCurrentAction;
	private final ArrayList<Action> mRemainingActions;

	public RepeatAction(Action action, int repeats) {
		mRemainingActions = new ArrayList<>(repeats);
		for(int i = 0; i < repeats; i++) {
			mRemainingActions.add(action);
		}
		mCurrentAction = null;
	}

	@Override
	public void start() {}

	@Override
	public void update() {
		if (mCurrentAction == null) {
			if (mRemainingActions.isEmpty()) {
				return;
			}

			mCurrentAction = mRemainingActions.remove(0);
			mCurrentAction.start();
		}

		mCurrentAction.update();

		if (mCurrentAction.isFinished()) {
			mCurrentAction.done();
			mCurrentAction = null;
		}
	}

	@Override
	public boolean isFinished() {
		return mRemainingActions.isEmpty() && mCurrentAction == null;
	}

	@Override
	public void done() {}
}
