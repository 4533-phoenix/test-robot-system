package frc.robot.actions;

import edu.wpi.first.math.trajectory.*;
import edu.wpi.first.math.trajectory.constraint.MaxVelocityConstraint;
import edu.wpi.first.math.trajectory.constraint.SwerveDriveKinematicsConstraint;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.SwerveSystem;
import frc.robot.Constants;
import frc.robot.Robot;

import java.util.ArrayList;
import java.util.Arrays;

public final class SwerveActions {
    public static ActionThread testSwerveAutonomous() {
        TrajectoryConfig config = new TrajectoryConfig(
            SwerveSystem.MAX_VELOCITY, 
            SwerveSystem.MAX_ACCELERATION
        )
        .addConstraint(
            new MaxVelocityConstraint(
                SwerveSystem.MAX_VELOCITY
            )
        )
        .addConstraint(
            new SwerveDriveKinematicsConstraint(
                Robot.swerveSystem.getSwerveKinematics(), 
                SwerveSystem.MAX_VELOCITY
            )
        );

        Pose2d startPose = Robot.swerveSystem.getSwervePose();

        ArrayList<Translation2d> trajectoryPoints = new ArrayList<Translation2d>(
            Arrays.asList(
                new Translation2d(startPose.getX() + 1, startPose.getY() + 1),
                new Translation2d(startPose.getX() + 2, startPose.getY() - 1)
            )
        );

        Pose2d endPose = new Pose2d(startPose.getX() + 3, startPose.getY(), startPose.getRotation());

        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            Robot.swerveSystem.getSwervePose(), 
            trajectoryPoints, 
            endPose, 
            config
        );

        Runnable trajectoryRunnable = new Runnable() {
            @Override
            public void run() {
                double time = trajectory.getTotalTimeSeconds();

                HolonomicDriveController swerveChassisController = Robot.swerveSystem.getSwerveChassisController();

                Timer timer = new Timer();
                timer.reset();
                timer.start();

                while (timer.get() <= time) {
                    Trajectory.State trajectoryState = trajectory.sample(timer.get());

                    ChassisSpeeds chassisSpeeds = swerveChassisController.calculate(
                        Robot.swerveSystem.getSwervePose(), 
                        trajectoryState,
                        trajectoryState.poseMeters.getRotation()
                    );

                    SwerveModuleState[] swerveModuleStates = Robot.swerveSystem.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);
                    
                    Robot.swerveSystem.drive(swerveModuleStates, false);
                }

                Robot.swerveSystem.drive(new SwerveModuleState[]{}, true);
            }
        };

        return new ActionThread(trajectoryRunnable, false, true, false, Robot.swerveSystem.getThreadLock());
    }

    public static ActionThread defaultSwerveDrive() {
        Runnable defaultSwerveDriveRunnable = new Runnable() {
            @Override
            public void run() {
                double leftStickX = Robot.driveControllerOne.getRawAxis(Constants.leftStickXAxis);
                double leftStickY = Robot.driveControllerOne.getRawAxis(Constants.leftStickYAxis);

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
                    rightStickX * SwerveSystem.MAX_ROTATIONAL_VELOCITY
                );

                SwerveModuleState[] swerveModuleStates = Robot.swerveSystem.getSwerveKinematics().toSwerveModuleStates(chassisSpeeds);

                Robot.swerveSystem.drive(swerveModuleStates, false);
            }
        };

        return new ActionThread(defaultSwerveDriveRunnable, true, false, false, Robot.swerveSystem.getThreadLock());
    }
}
