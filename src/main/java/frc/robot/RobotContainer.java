package frc.robot;

import frc.robot.actions.ActionThread;
import frc.robot.actions.SwerveActions;

import java.util.Map;

public final class RobotContainer {
    private Map<String, ActionThread> autonomousActionMap = Map.ofEntries(
        Map.entry("testSwerveAutonomous", SwerveActions.testSwerveAutonomous())
    );

    private void queryInitialActions() {
        Robot.swerveSystem.queryInitialActions();
    }

    private void queryEventActions() {}

    public RobotContainer() {
        this.queryInitialActions();

        this.queryEventActions();
    }

    public ActionThread getAutonomousAction(String autonomousActionKey) {
        return this.autonomousActionMap.get(autonomousActionKey);
    }
}
