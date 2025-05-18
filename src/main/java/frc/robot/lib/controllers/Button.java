package frc.robot.lib.controllers;

import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import java.util.function.Supplier;

import frc.robot.ActionExecutor;
import frc.robot.actions.Action;
import frc.robot.actions.HoldableAction;
import frc.robot.lib.loops.*;

public class Button {
    private final Supplier<Boolean> mButtonSupplier;
    private final Optional<ActionExecutor> mActionExecutor;
    private final Looper mLooper;

    private Runnable onPressed = () -> {}; 
    private Runnable onClicked = () -> {}; 
    private Runnable onHeld = () -> {};
    private Runnable onReleased = () -> {};

    private boolean cached = false;

    public Button(Supplier<Boolean> buttonSupplier, Looper looper, Optional<ActionExecutor> actionExecutor) {
        mButtonSupplier = buttonSupplier;
        mActionExecutor = actionExecutor;
        mLooper = looper;

        mLooper.register(new Loop() {
            @Override public void onStart(double timestamp) {}

            @Override public void onLoop(double timestamp) {
                boolean current = mButtonSupplier.get();
                boolean clicked = !cached && current;
                boolean released =  cached && !current;
                cached = current;

                if (clicked) {
                    onClicked.run();
                }
                if (current) {
                    onPressed.run();
                    if (clicked) {
                        onHeld.run();
                    }
                }
                if (released) { 
                    onReleased.run();
                }
            }

            @Override public void onStop(double timestamp) {}
        });
    }

    public Button(Supplier<Boolean> supplier, Looper looper) {
        this(supplier, looper, Optional.empty());
    }

    public void setPressed(Runnable runnable) {
        onPressed = runnable;
    }

    public void setClicked(Runnable runnable) {
        onClicked = runnable;
    }

    public void setHeld(Runnable runnable) {
        onHeld = runnable;
    }

    public void setReleased(Runnable runnable) {
        onReleased = runnable;
    }

    public void setAction(HoldableAction action) {
        mActionExecutor.ifPresent(actionExecutor -> {
            setClicked(() -> actionExecutor.runAction(action));
            setReleased(action::finish);
        });
    }

    public void setAction(Action action) {
        mActionExecutor.ifPresent(actionExecutor -> {
            setClicked(() -> actionExecutor.runAction(action));
        });
    }

    public void setActionWhenReleased(Action action) {
        mActionExecutor.ifPresent(actionExecutor -> {
            setReleased(() -> actionExecutor.runAction(action));
        });
    }
}

