package frc.robot.subsystems;

import com.swervedrivespecialties.swervelib.SwerveModule;
import com.swervedrivespecialties.swervelib.Mk4SwerveModuleHelper;
import com.swervedrivespecialties.swervelib.SdsModuleConfigurations;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class SwerveSystem extends Subsystem {
    private SwerveModule[] swerveModules;

    private ChassisSpeeds chassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);
    private SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics();
    private SwerveDrivePoseEstimator swervePoseEstimator;

    private HolonomicDriveController swerveController = new HolonomicDriveController(
        new PIDController(1.0, 0.0, 0.0),
        new PIDController(1.0, 0.0, 0.0),
        new ProfiledPIDController(1.0, 0.0, 0.0,
            new TrapezoidProfile.Constraints(6.28, 3.14))
    );

    public SwerveSystem() {
        SwerveModule frontLeftSwerveModule;
        SwerveModule frontRightSwerveModule;
        SwerveModule backLeftSwerveModule;
        SwerveModule backRightSwerveModule;
    }
}
