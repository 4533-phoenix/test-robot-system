package frc.libs.sdsLib;

public interface DriveController {
    void setReferenceVoltage(double voltage);

    double getStateVelocity();
}
