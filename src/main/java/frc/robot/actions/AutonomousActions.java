package frc.robot.actions;

import java.util.ArrayList;
import java.util.Arrays;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.constraint.*;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.Timer;

import frc.robot.subsystems.SwerveSystem;
import frc.robot.Robot;

public final class AutonomousActions {
    public static Action testSwerveAutonomous() {
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

        Trajectory swerveTrajectory = TrajectoryGenerator.generateTrajectory(
            Robot.swerveSystem.getSwervePose(), 
            trajectoryPoints, 
            endPose, 
            config
        );

        Runnable trajectoryRunnable = new Runnable() {
            @Override
            public void run() {
                double time = swerveTrajectory.getTotalTimeSeconds();

                Timer timer = new Timer();
                timer.reset();
                timer.start();

                while (timer.get() <= time) {
                    HolonomicDriveController swerveChassisController = Robot.swerveSystem.getSwerveChassisController();

                    Trajectory.State trajectoryState = swerveTrajectory.sample(timer.get());

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

        return new Action(trajectoryRunnable, true, false, Robot.swerveSystem.getSubsystemThreadLock());
    }
}
