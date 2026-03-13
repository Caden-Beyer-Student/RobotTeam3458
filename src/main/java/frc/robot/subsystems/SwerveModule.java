package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Abstract interface for a single swerve module (drive + steer)
 */
public interface SwerveModule {

    /** Set the desired wheel state (speed + angle) */
    void setDesiredState(SwerveModuleState state);

    /** Get current wheel angle */
    Rotation2d getAngle();

    /** Get current wheel speed (m/s) */
    double getVelocityMetersPerSecond();

    /** Reset encoders / angle */
    void resetToAbsolute();

    /** Get current state (speed + angle) */
    SwerveModuleState getState();
}