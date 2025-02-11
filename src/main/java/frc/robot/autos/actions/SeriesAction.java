package frc.robot.autos.actions;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

/**
 * Executes one action at a time. Useful as a member of {@link ParallelAction}
 */
public class SeriesAction implements Action {
	private Action mCurrentAction;
	private ArrayList<Action> mActionList;
	private ArrayList<Action> mRemainingActions;

	public SeriesAction(List<Action> actions) {
		mActionList = new ArrayList<>(actions.size());
		mActionList.addAll(actions);
		mCurrentAction = null;
	}

	public SeriesAction(Action... actions) {
		this(Arrays.asList(actions));
	}

	@Override
	public void start() {
		mRemainingActions = new ArrayList<>(mActionList);
		mCurrentAction = null;
		System.out.println("Series Action running!");
	}

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
