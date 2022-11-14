package frc.robot.subsystems;

import frc.robot.actions.ActionDeque;
import frc.robot.actions.Action;

public class Subsystem {
    private final Action periodicAction = new Action(
        () -> { this.periodic(); }, 
        false, 
        false
    );

    private final Action loggingAction = new Action(
        () -> { this.log(); }, 
        false, 
        false
    );

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
