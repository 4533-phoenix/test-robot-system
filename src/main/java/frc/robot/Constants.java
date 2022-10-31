package frc.robot;

public final class Constants {
    // TODO: Get the correct motor ports
    // Swerve module motor ports
    public static final int frontLeftDriveMotorPort = 0;
    public static final int frontLeftSteerMotorPort = 1;
    
    public static final int frontRightDriveMotorPort = 2;
    public static final int frontRightSteerMotorPort = 3;

    public static final int backLeftDriveMotorPort = 4;
    public static final int backLeftSteerMotorPort = 5;

    public static final int backRightDriveMotorPort = 6;
    public static final int backRightSteerMotorPort = 7;

    // TODO: Get the correct CANCoder ports
    // Swerve module CANCoder ports
    public static final int frontLeftSteerEncoderPort = 8;
    public static final int frontRightSteerEncoderPort = 9;
    public static final int backLeftSteerEncoderPort = 10;
    public static final int backRightSteerEncoderPort = 11;

    // Swerve module indices
    public static final int frontLeftSwerveModuleIndex = 0;
    public static final int frontRightSwerveModuleIndex = 1;
    public static final int backLeftSwerveModuleIndex = 2;
    public static final int backRightSwerveModuleIndex = 3;

    // TODO: Calculate these
    // Swerve module steer motor offsets
    public static final double frontLeftSteerMotorOffset = 0.0;
    public static final double frontRightSteerMotorOffset = 0.0;
    public static final double backLeftSteerMotorOffset = 0.0;
    public static final double backRightSteerMotorOffset = 0.0;

    // Swerve module wheel diameter
    public static final double swerveModuleWheelDiameter = 0.1016; // meters

    // Swerve module distance from center of robot (in both x and y direction)
    public static final double swerveModuleDistanceFromCenter = 0.28575; // meters
}
