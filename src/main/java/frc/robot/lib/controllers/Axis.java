package frc.robot.lib.controllers;

import java.lang.StackWalker.Option;
import java.util.Optional;
import java.util.function.Consumer;
import java.util.function.Supplier;

import frc.robot.ActionExecutor;
import frc.robot.Constants;
import frc.robot.actions.Action;
import frc.robot.lib.loops.*;
import frc.robot.lib.util.Util;

public class Axis {
    private Supplier<Double> mAxisSupplier;
    private Looper mLooper;
    private Consumer<Double> mConsumer = (Double x) -> {};
    private double mDeadband = 0.0;

    public Axis(Supplier<Double> axisSupplier, Looper looper) {
        mAxisSupplier = axisSupplier;
        mLooper = looper;

        mLooper.register(new Loop() {
            @Override public void onStart(double timestamp) {}

            @Override public void onLoop(double timestamp) {
                double current = mAxisSupplier.get();
                mConsumer.accept(Util.deadBand(current, mDeadband));
            }

            @Override public void onStop(double timestamp) {}
        });
    }

    public void bind(Consumer<Double> consumer) {
        mConsumer = consumer;
        mDeadband = Constants.STICK_DEADBAND;
    }

    public void bind(Consumer<Double> consumer, double deadband) {
        mConsumer = consumer;
        mDeadband = deadband;
    }

    public void bindRaw(Consumer<Double> consumer) {
        mConsumer = consumer;
    }

    public Button bindAsButton(double threshold) {
        return new Button(() -> { return mAxisSupplier.get() > threshold; }, mLooper);
    }

    public Button bindAsButton(double threshold, ActionExecutor actionExecutor) {
        return new Button(() -> { return mAxisSupplier.get() > threshold; }, mLooper, Optional.ofNullable(actionExecutor));
    }

    public Supplier<Double> getSupplier() {
        return mAxisSupplier;
    }
}
