package frc.robot;

import frc.robot.actions.Action;
import frc.robot.actions.AutonomousActions;

import java.util.Map;

public final class RobotContainer {
    private Map<String, Action> autonomousActionMap = Map.ofEntries(
        Map.entry("testSwerveAutonomous", AutonomousActions.testSwerveAutonomous())
    );

    private void queryInitialActions() {
        Robot.swerveSystem.queryInitialActions();
    }

    private void queryEventActions() {}

    public RobotContainer() {
        this.queryInitialActions();

        this.queryEventActions();
    }

    public Action getAutonomousAction(String autonomousActionKey) {
        return this.autonomousActionMap.get(autonomousActionKey);
    }
}
