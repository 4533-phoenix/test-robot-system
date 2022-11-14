package frc.robot.subsystems;

import frc.robot.actions.ActionDeque;

import java.util.concurrent.locks.ReentrantLock;

import frc.robot.actions.Action;

public class Subsystem {
    private ReentrantLock subsystemThreadLock = new ReentrantLock(true);

    private final Action periodicAction = new Action(
        () -> { this.periodic(); }, 
        false, 
        false,
        null
    );

    private final Action loggingAction = new Action(
        () -> { this.log(); }, 
        false, 
        false,
        null
    );

    public ReentrantLock getSubsystemThreadLock() {
        return this.subsystemThreadLock;
    }

    protected final Action getPeriodicAction() {
        return this.periodicAction;
    }

    protected final Action getLoggingAction() {
        return this.loggingAction;
    }

    public void log() {}

    public void periodic() {}

    public void queryInitialActions() {
        ActionDeque.getInstance().pushBack(
            this.periodicAction,
            this.loggingAction
        );
    }
}
