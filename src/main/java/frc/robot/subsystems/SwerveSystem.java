package frc.robot.subsystems;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.kauailabs.navx.frc.AHRS;

import frc.libs.sdsLib.SwerveModule;
import frc.libs.sdsLib.Mk4SwerveModuleHelper;
import frc.libs.sdsLib.Mk4SwerveModuleHelper.GearRatio;

import frc.robot.Constants;
import frc.robot.actions.ActionDeque;
import frc.robot.actions.SwerveActions;

import static java.lang.Math.*;

public final class SwerveSystem extends Subsystem {
    public static final double MAX_VELOCITY = 0.5; // meters/second
    public static final double MAX_ACCELERATION = MAX_VELOCITY / 2; // meters/second^2

    public static final double MAX_ROTATIONAL_VELOCITY = 2 * PI; // radians/second
    public static final double MAX_ROTATIONAL_ACCELERATION = PI; // radians/second^2

    public static final double MAX_VOLTAGE = 12.0;

    private SwerveModule[] swerveModules = new SwerveModule[4];

    private AHRS navX = new AHRS(SPI.Port.kMXP);

    private static double fieldRelativeAngleOffset = 0.0;

    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
        new Translation2d(-Constants.swerveModuleDistanceFromCenter, Constants.swerveModuleDistanceFromCenter),
        new Translation2d(Constants.swerveModuleDistanceFromCenter, Constants.swerveModuleDistanceFromCenter),
        new Translation2d(-Constants.swerveModuleDistanceFromCenter, -Constants.swerveModuleDistanceFromCenter),
        new Translation2d(Constants.swerveModuleDistanceFromCenter, -Constants.swerveModuleDistanceFromCenter)
    );

    private SwerveDrivePoseEstimator swervePoseEstimator;

    private Pose2d swervePose;

    // Holonomic PID Constants' output is velocity based on input of displacement error 
    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_X_VELOCITY_KP = (0.2 * MAX_VELOCITY) / MAX_VELOCITY;
    public static final double SWERVE_CHASSIS_X_VELOCITY_KI = 0.0;
    public static final double SWERVE_CHASSIS_X_VELOCITY_KD = SWERVE_CHASSIS_X_VELOCITY_KP * 10;

    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_Y_VELOCITY_KP = (0.2 * MAX_VELOCITY) / MAX_VELOCITY;
    public static final double SWERVE_CHASSIS_Y_VELOCITY_KI = 0.0;
    public static final double SWERVE_CHASSIS_Y_VELOCITY_KD = SWERVE_CHASSIS_Y_VELOCITY_KP * 10;

    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_OMEGA_KP = (0.2 * MAX_ROTATIONAL_VELOCITY) / MAX_ROTATIONAL_VELOCITY;
    public static final double SWERVE_CHASSIS_OMEGA_KI = 0.0;
    public static final double SWERVE_CHASSIS_OMEGA_KD = SWERVE_CHASSIS_OMEGA_KP * 10;

    private HolonomicDriveController swerveChassisController = new HolonomicDriveController(
        new PIDController(
            SWERVE_CHASSIS_X_VELOCITY_KP, 
            SWERVE_CHASSIS_X_VELOCITY_KI, 
            SWERVE_CHASSIS_X_VELOCITY_KD
        ),
        new PIDController(
            SWERVE_CHASSIS_Y_VELOCITY_KP, 
            SWERVE_CHASSIS_Y_VELOCITY_KI, 
            SWERVE_CHASSIS_Y_VELOCITY_KD
        ),
        new ProfiledPIDController(
            SWERVE_CHASSIS_OMEGA_KP, 
            SWERVE_CHASSIS_OMEGA_KI, 
            SWERVE_CHASSIS_OMEGA_KD,
            new TrapezoidProfile.Constraints(MAX_ROTATIONAL_VELOCITY, MAX_ROTATIONAL_ACCELERATION)
        )
    );

    // TODO: Calculate these using SysID
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KS = 1.0;
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KV = 1.0;
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KA = 1.0;

    // Swerve Drive Motor PID Constants' output is voltage based on input of velocity error
    // TODO: Calculate these
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_KP = (0.2 * MAX_VOLTAGE) / MAX_VELOCITY;
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_KI = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_VELOCITY_KD = SWERVE_DRIVE_MOTOR_VELOCITY_KP * 10;

    private SimpleMotorFeedforward swerveModuleDriveMotorFeedForward = new SimpleMotorFeedforward(
        SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KS,
        SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KV,
        SWERVE_DRIVE_MOTOR_VELOCITY_FEEDFORWARD_KA
    );

    private ProfiledPIDController swerveModuleDriveMotorController = new ProfiledPIDController(
        SWERVE_DRIVE_MOTOR_VELOCITY_KP, 
        SWERVE_DRIVE_MOTOR_VELOCITY_KI, 
        SWERVE_DRIVE_MOTOR_VELOCITY_KD, 
        new TrapezoidProfile.Constraints(MAX_VELOCITY, MAX_ACCELERATION)
    );

    public SwerveSystem() {
        navX.reset();

        SwerveModule frontLeftSwerveModule = Mk4SwerveModuleHelper.createNeo(
            GearRatio.L2, 
            Constants.frontLeftDriveMotorPort, 
            Constants.frontLeftSteerMotorPort, 
            Constants.frontLeftSteerEncoderPort, 
            Constants.frontLeftSteerMotorOffset
        );

        SwerveModule frontRightSwerveModule = Mk4SwerveModuleHelper.createNeo(
            GearRatio.L2, 
            Constants.frontRightDriveMotorPort, 
            Constants.frontRightSteerMotorPort, 
            Constants.frontRightSteerEncoderPort, 
            Constants.frontRightSteerMotorOffset
        );
        
        SwerveModule backLeftSwerveModule = Mk4SwerveModuleHelper.createNeo(
            GearRatio.L2,
            Constants.backLeftDriveMotorPort, 
            Constants.backLeftSteerMotorPort, 
            Constants.backLeftSteerEncoderPort, 
            Constants.backLeftSteerMotorOffset
        );
        
        SwerveModule backRightSwerveModule = Mk4SwerveModuleHelper.createNeo(
            GearRatio.L2, 
            Constants.backRightDriveMotorPort, 
            Constants.backRightSteerMotorPort, 
            Constants.backRightSteerEncoderPort, 
            Constants.backRightSteerMotorOffset
        );

        this.swerveModules[Constants.frontLeftSwerveModuleIndex] = frontLeftSwerveModule;
        this.swerveModules[Constants.frontRightSwerveModuleIndex] = frontRightSwerveModule;
        this.swerveModules[Constants.backLeftSwerveModuleIndex] = backLeftSwerveModule;
        this.swerveModules[Constants.backRightSwerveModuleIndex] = backRightSwerveModule;

        this.swervePose = new Pose2d();

        // TODO: Have initial position be changeable
        this.swervePoseEstimator = new SwerveDrivePoseEstimator(
            Rotation2d.fromDegrees(-navX.getAngle()), 
            this.swervePose, 
            this.swerveKinematics, 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), 
            new MatBuilder<>(Nat.N1(), Nat.N1()).fill(0.01), 
            new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01)
        );
    }

    public void drive(SwerveModuleState[] swerveModuleStates, boolean snap) {
        if (snap) {
            this.swerveModules[Constants.frontLeftSwerveModuleIndex].set(0.0, (PI / 4));
            this.swerveModules[Constants.frontRightSwerveModuleIndex].set(0.0, (3 * PI / 4));
            this.swerveModules[Constants.backLeftSwerveModuleIndex].set(0.0, (-PI / 4));
            this.swerveModules[Constants.backRightSwerveModuleIndex].set(0.0, (-3 * PI / 4));
        }
        else {
            SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, MAX_VELOCITY);

            for (int i = 0; i < 4; i++) {
                SwerveModule swerveModule = this.swerveModules[i];

                double speed = swerveModuleStates[i].speedMetersPerSecond;
                double angle = swerveModuleStates[i].angle.getRadians();

                // TODO: Consider using an acceleration value for the feedforward calculation
                swerveModule.set(
                    swerveModuleDriveMotorController.calculate(swerveModule.getDriveVelocity(), speed)
                    + swerveModuleDriveMotorFeedForward.calculate(speed), 
                    angle + fieldRelativeAngleOffset
                );
            }
        }
    }

    public SwerveModuleState[] calculateStates(Translation2d movementVector, double steerVelocity) {
        /* 
         * For constructor, vx is forward with forward being the positive direction, 
         * vy is horizontal with left being the positive direction, and omega is
         * the rotational velocity with counterclockwise being the positive direction
        */
        ChassisSpeeds chassisSpeeds = new ChassisSpeeds(movementVector.getY(), -movementVector.getX(), -steerVelocity);

        SwerveModuleState[] swerveModuleStates = this.swerveKinematics.toSwerveModuleStates(chassisSpeeds);

        return swerveModuleStates;
    }

    public SwerveDriveKinematics getSwerveKinematics() {
        return this.swerveKinematics;
    }

    public HolonomicDriveController getSwerveChassisController() {
        return this.swerveChassisController;
    }
    
    public Pose2d getSwervePose() {
        return this.swervePose;
    }

    @Override
    public void log() {
        SmartDashboard.putNumber("Swerve Pose X", this.swervePose.getX());
        SmartDashboard.putNumber("Swerve Pose Y", this.swervePose.getY());
        SmartDashboard.putNumber("Swerve Pose Angle", this.swervePose.getRotation().getDegrees());

        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = new SwerveModuleState(
                this.swerveModules[i].getDriveVelocity(), 
                new Rotation2d(this.swerveModules[i].getSteerAngle())
            );
        }

        ChassisSpeeds chassisSpeeds = this.swerveKinematics.toChassisSpeeds(
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        );

        SmartDashboard.putNumber("Swerve X Velocity", -chassisSpeeds.vyMetersPerSecond);
        SmartDashboard.putNumber("Swerve Y Velocity", chassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("Swerve Rotational Velocity", -chassisSpeeds.omegaRadiansPerSecond);
    }

    @Override
    public void periodic() {
        fieldRelativeAngleOffset = toRadians(-navX.getYaw());
        
        SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];

        for (int i = 0; i < 4; i++) {
            swerveModuleStates[i] = new SwerveModuleState(
                this.swerveModules[i].getDriveVelocity(), 
                new Rotation2d(this.swerveModules[i].getSteerAngle())
            );
        }

        this.swervePose = this.swervePoseEstimator.update(
            Rotation2d.fromDegrees(-navX.getAngle()),
            swerveModuleStates[0],
            swerveModuleStates[1],
            swerveModuleStates[2],
            swerveModuleStates[3]
        );

        System.out.println("Front Left Offset: " + swerveModules[0].getSteerAngle());
    }

    @Override
    public void queryInitialActions() {
        super.queryInitialActions();

        ActionDeque.getInstance().pushBack(
            SwerveActions.defaultSwerveDrive()
        );
    }
}
