package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4ModuleConfiguration;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper.GearRatio;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

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

import com.kauailabs.navx.frc.AHRS;

import frc.robot.Constants;

import static java.lang.Math.*;

public class SwerveSystem extends Subsystem {
    public static final double MAX_VELOCITY = 26.5988; // meters/second
    public static final double MAX_ACCELERATION = 13.2994; // meters/second^2

    public static final double MAX_ROTATIONAL_VELOCITY = 2 * PI; // radians/second
    public static final double MAX_ROTATIONAL_ACCELERATION = PI; // radians/second^2

    private SwerveModule[] swerveModules;

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

    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_X_KP = 1.0;
    public static final double SWERVE_CHASSIS_X_KI = 0.0;
    public static final double SWERVE_CHASSIS_X_KD = 0.0;

    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_Y_KP = 1.0;
    public static final double SWERVE_CHASSIS_Y_KI = 0.0;
    public static final double SWERVE_CHASSIS_Y_KD = 0.0;

    // TODO: Calculate these
    public static final double SWERVE_CHASSIS_THETA_KP = 1.0;
    public static final double SWERVE_CHASSIS_THETA_KI = 0.0;
    public static final double SWERVE_CHASSIS_THETA_KD = 0.0;

    private HolonomicDriveController swerveChassisController = new HolonomicDriveController(
        new PIDController(
            SWERVE_CHASSIS_X_KP, 
            SWERVE_CHASSIS_X_KI, 
            SWERVE_CHASSIS_X_KD
        ),
        new PIDController(
            SWERVE_CHASSIS_Y_KP, 
            SWERVE_CHASSIS_Y_KI, 
            SWERVE_CHASSIS_Y_KD
        ),
        new ProfiledPIDController(
            SWERVE_CHASSIS_THETA_KP, 
            SWERVE_CHASSIS_THETA_KI, 
            SWERVE_CHASSIS_THETA_KD,
            new TrapezoidProfile.Constraints(MAX_ROTATIONAL_VELOCITY, MAX_ROTATIONAL_ACCELERATION)
        )
    );

    // TODO: Calculate these using SysID
    public static final double SWERVE_DRIVE_MOTOR_FEEDFORWARD_KS = 1.0;
    public static final double SWERVE_DRIVE_MOTOR_FEEDFORWARD_KV = 1.0;
    public static final double SWERVE_DRIVE_MOTOR_FEEDFORWARD_KA = 1.0;

    // TODO: Calculate these
    public static final double SWERVE_DRIVE_MOTOR_KP = 1.0;
    public static final double SWERVE_DRIVE_MOTOR_KI = 0.0;
    public static final double SWERVE_DRIVE_MOTOR_KD = 0.0;

    private SimpleMotorFeedforward swerveModuleDriveMotorFeedForward = new SimpleMotorFeedforward(
        SWERVE_DRIVE_MOTOR_FEEDFORWARD_KS,
        SWERVE_DRIVE_MOTOR_FEEDFORWARD_KV,
        SWERVE_DRIVE_MOTOR_FEEDFORWARD_KA
    );

    private ProfiledPIDController swerveModuleDriveMotorController = new ProfiledPIDController(
        SWERVE_DRIVE_MOTOR_KP, 
        SWERVE_DRIVE_MOTOR_KI, 
        SWERVE_DRIVE_MOTOR_KD, 
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
            this.swerveModules[Constants.frontLeftSwerveModuleIndex].set(0.0, (PI / 4) + fieldRelativeAngleOffset);
            this.swerveModules[Constants.frontRightSwerveModuleIndex].set(0.0, (3 * PI / 4) + fieldRelativeAngleOffset);
            this.swerveModules[Constants.backLeftSwerveModuleIndex].set(0.0, (-PI / 4) + fieldRelativeAngleOffset);
            this.swerveModules[Constants.backRightSwerveModuleIndex].set(0.0, (-3 * PI / 4) + fieldRelativeAngleOffset);
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

    // TODO: Create an action thread that runs this
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
    }
}
