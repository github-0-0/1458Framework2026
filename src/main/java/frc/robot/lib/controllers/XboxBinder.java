package frc.robot.lib.controllers;

import java.util.HashMap;
import java.util.List;
import java.util.Objects;
import java.util.Optional;
import java.util.concurrent.ConcurrentHashMap;
import java.util.function.Consumer;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import java.util.stream.Collectors;

import edu.wpi.first.wpilibj.XboxController;
import frc.robot.ActionExecutor;
import frc.robot.actions.Action;
import frc.robot.actions.HoldableAction;
import frc.robot.lib.loops.Loop;
import frc.robot.lib.loops.Looper;

import java.util.Arrays;
import java.util.EnumMap;

public class XboxBinder {
    public XboxController mXboxController;
    public Looper mLooper;
    private final EnumMap<Buttons, Button> mButtons;
    private final EnumMap<Axes, Axis> mAxes;
    public ActionExecutor mActionExecutor;

    public enum When {
        PRESSED,
        CLICKED,
        HELD,
        RELEASED
    }

    public enum Buttons {
        A, B, X, Y,
        LEFT_BUMPER, RIGHT_BUMPER,
        BACK, START,
        LEFT_JOYSTICK, RIGHT_JOYSTICK
    }

    public enum Axes {
        LEFT_JOYSTICK_X, LEFT_JOYSTICK_Y,
        RIGHT_JOYSTICK_X, RIGHT_JOYSTICK_Y,
        LEFT_TRIGGER, RIGHT_TRIGGER
    }

    /**
     * An object to bind runnables or actions to an XboxController.
     * Call {@code start()} after initializing bindings.
     * @param xboxController the XboxController to bind to
     */
    public XboxBinder(XboxController xboxController, Looper looper) {
        this(xboxController, looper, null);
    }

    /**
     * An object to bind runnables or actions to an XboxController.
     * Call {@code start()} after initializing bindings.
     * @param xboxController the XboxController to bind to
     * @param actionExecutor the current action executor
     */
    public XboxBinder(XboxController xboxController, Looper looper, ActionExecutor actionExecutor) {
        mXboxController = xboxController;
        mActionExecutor = actionExecutor;
        mLooper = looper;
        mButtons = new EnumMap<>(Buttons.class);
        mAxes = new EnumMap<>(Axes.class);

        // Button mappings
        mButtons.put(Buttons.A, new Button(mXboxController::getAButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.B, new Button(mXboxController::getBButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.X, new Button(mXboxController::getXButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.Y, new Button(mXboxController::getYButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.LEFT_BUMPER, new Button(mXboxController::getLeftBumperButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.RIGHT_BUMPER, new Button(mXboxController::getRightBumperButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.BACK, new Button(mXboxController::getBackButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.START, new Button(mXboxController::getStartButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.LEFT_JOYSTICK, new Button(mXboxController::getLeftStickButton, mLooper, Optional.ofNullable(actionExecutor)));
        mButtons.put(Buttons.RIGHT_JOYSTICK, new Button(mXboxController::getRightStickButton, mLooper, Optional.ofNullable(actionExecutor)));

        // Axis mappings
        mAxes.put(Axes.LEFT_JOYSTICK_X, new Axis(mXboxController::getLeftX, mLooper));
        mAxes.put(Axes.LEFT_JOYSTICK_Y, new Axis(mXboxController::getLeftY, mLooper));
        mAxes.put(Axes.RIGHT_JOYSTICK_X, new Axis(mXboxController::getRightX, mLooper));
        mAxes.put(Axes.RIGHT_JOYSTICK_Y, new Axis(mXboxController::getRightY, mLooper));
        mAxes.put(Axes.LEFT_TRIGGER, new Axis(mXboxController::getLeftTriggerAxis, mLooper));
        mAxes.put(Axes.RIGHT_TRIGGER, new Axis(mXboxController::getRightTriggerAxis, mLooper));
    }

    /**
     * Bind a Runnable to a control event.
     * @param key the button or axis enum
     * @param when the event type
     * @param runnable the action to run
     */
    public void bind(Buttons key, When when, Runnable runnable) {
        Button btn = mButtons.get(key);
        if (btn == null) {
            throw new IllegalArgumentException("No button mapped to " + key);
        }
        switch (when) {
            case PRESSED:
                btn.setPressed(runnable);
                break;
            case CLICKED:
                btn.setClicked(runnable);
                break;
            case HELD:
                btn.setHeld(runnable);
                break;
            case RELEASED:
                btn.setReleased(runnable);
                break;
        }
    }

    public void bind(Buttons key, When when, Action action) {
        Button btn = mButtons.get(key);
        if (btn == null) {
            throw new IllegalArgumentException("No button mapped to " + key);
        }
        switch (when) {
            case CLICKED: btn.setAction(action); break;
            case RELEASED: btn.setActionWhenReleased(action); break;
            default: throw new IllegalArgumentException("Actions must be bound to edge cases.");
        }
    }

    /**
     * Bind a Runnable to a control event.
     * @param id the raw button id
     * @param when the event type
     * @param runnable the action to run
     */
    public void bindToRawButton(int id, When when, Runnable runnable) {
        Button btn = new Button(() -> { return mXboxController.getRawButton(id); }, mLooper, Optional.ofNullable(mActionExecutor));

        switch (when) {
            case PRESSED:
                btn.setPressed(runnable);
                break;
            case CLICKED:
                btn.setClicked(runnable);
                break;
            case HELD:
                btn.setHeld(runnable);
                break;
            case RELEASED:
                btn.setReleased(runnable);
                break;
        }
    }

    public void bindToRawButton(int id, When when, Action action) {
        Button btn = new Button(() -> { return mXboxController.getRawButton(id); }, mLooper, Optional.ofNullable(mActionExecutor));

        switch (when) {
            case CLICKED: btn.setAction(action); break;
            case RELEASED: btn.setActionWhenReleased(action); break;
            default: throw new IllegalArgumentException("Actions must be bound to edge cases.");
        }
    }

    public void bind(Axes key, Consumer<Double> consumer) {
        Axis axis = mAxes.get(key);
        if (axis == null) {
            throw new IllegalArgumentException("No axis mapped to " + key);
        }
        axis.bind(consumer);
    }

    public void bindPressed(Buttons key, Runnable runnable) { bind(key, When.PRESSED, runnable); }
    public void bindClicked(Buttons key, Runnable runnable) { bind(key, When.CLICKED, runnable); }
    public void bindHeld(Buttons key, Runnable runnable) { bind(key, When.HELD, runnable); }
    public void bindReleased(Buttons key, Runnable runnable) { bind(key, When.RELEASED, runnable); }

    public void bindAction(Buttons key, Action action) { bind(key, When.CLICKED, action); }

    public void bindToAxisRaw(Axes key, Consumer<Double> consumer) { mAxes.get(key).bindRaw(consumer); }
    public void bindToAxisWithDeadband(Axes key, Consumer<Double> consumer, double deadband) { mAxes.get(key).bind(consumer, deadband); }

    public void bindAxisAsButton(Axes key, double threshold, When when, Runnable runnable) {
        Button wrapper = mAxes.get(key).bindAsButton(threshold, mActionExecutor);
        switch (when) {
            case PRESSED: wrapper.setPressed(runnable); break;
            case CLICKED: wrapper.setClicked(runnable); break;
            case HELD: wrapper.setHeld(runnable); break;
            case RELEASED: wrapper.setReleased(runnable); break;
        }
    }

    public void bindAxisAsButton(Axes key, double threshold, When when, Action action) {
        Button wrapper = mAxes.get(key).bindAsButton(threshold, mActionExecutor);
        switch (when) {
            case CLICKED: wrapper.setAction(action); break;
            case RELEASED: wrapper.setActionWhenReleased(action); break;
            default: throw new IllegalArgumentException("Actions must be bound to edge cases.");
        }
    }

     /**
     * Bind a Consumer to the current values of multiple axes.
     * @param consumer receives a List of Double values, in the same order as the axes you pass
     * @param keys     one or more Axes to poll
     */
    public void bindAxes(Consumer<List<Double>> consumer, Axes... keys) {
        List<Supplier<Double>> suppliers = Arrays.stream(keys)
        .map(key -> {
            Axis axis = mAxes.get(key);
            if (axis == null) throw new IllegalArgumentException("No axis for " + key);
            return axis.getSupplier();
        })
        .collect(Collectors.toList());

        mLooper.register(new Loop() {
            @Override
            public void onStart(double timestamp) {}
            
            @Override
            public void onLoop(double timestamp) {
                List<Double> values = suppliers.stream()
                    .mapToDouble(s -> s.get())   
                    .boxed()                     
                    .collect(Collectors.toList());

                consumer.accept(values);
            }

            @Override
            public void onStop(double timestamp) {}
        });
           
    }
  
    public void start() {
        mLooper.start();
    }
}
