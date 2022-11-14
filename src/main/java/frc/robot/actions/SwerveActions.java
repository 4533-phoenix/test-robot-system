package frc.robot.actions;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;

import frc.robot.subsystems.SwerveSystem;
import frc.robot.Constants;
import frc.robot.Robot;

public final class SwerveActions {
    public static Action defaultSwerveDrive() {
        Runnable defaultSwerveDriveRunnable = new Runnable() {
            @Override
            public void run() {
                double leftStickX = Robot.driveControllerOne.getRawAxis(Constants.leftStickXAxis);
                double leftStickY = -Robot.driveControllerOne.getRawAxis(Constants.leftStickYAxis);

                double rightStickX = Robot.driveControllerOne.getRawAxis(Constants.rightStickXAxis);

                if (
                    (leftStickX >= -Constants.maxStickOffset && leftStickX <= Constants.maxStickOffset) &&
                    (leftStickY >= -Constants.maxStickOffset && leftStickY <= Constants.maxStickOffset) &&
                    (rightStickX >= -Constants.maxStickOffset && rightStickX <= Constants.maxStickOffset)
                ) {
                    Robot.swerveSystem.drive(new SwerveModuleState[]{}, true);

                    return;
                }

                ChassisSpeeds chassisSpeeds = new ChassisSpeeds(
                    leftStickY * SwerveSystem.MAX_VELOCITY, 
                    -leftStickX * SwerveSystem.MAX_VELOCITY,
                    -rightStickX * SwerveSystem.MAX_ROTATIONAL_VELOCITY
                );

                SwerveModuleState[] swerveModuleStates = Robot.swerveSystem.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);

                Robot.swerveSystem.drive(swerveModuleStates, false);
            }
        };

        return new Action(defaultSwerveDriveRunnable, false, false, null);
    }
}
