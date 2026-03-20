package frc.robot.util;

import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Publishes swerve module states to NetworkTables
 * so AdvantageScope can visualize them.
 *
 * This is SIMULATION / TELEMETRY ONLY.
 * It does NOT affect robot behavior.
 */
public class SwerveModuleStates {

    private final NetworkTable table;

    public SwerveModuleStates() {
        // AdvantageScope convention: put swerve data under /Swerve
        table = NetworkTableInstance.getDefault().getTable("Swerve");
    }

    /**
     * Publish all four module states
     */
    public void publish(
            SwerveModuleState frontLeft,
            SwerveModuleState frontRight,
            SwerveModuleState backLeft,
            SwerveModuleState backRight) {

        publishModule("FrontLeft", frontLeft);
        publishModule("FrontRight", frontRight);
        publishModule("BackLeft", backLeft);
        publishModule("BackRight", backRight);
    }

    /**
     * Publish a single module's state
     */
    private void publishModule(String name, SwerveModuleState state) {
        NetworkTable moduleTable = table.getSubTable(name);

        // Speed in meters per second
        moduleTable.getEntry("speed").setDouble(state.speedMetersPerSecond);

        // Angle in radians (AdvantageScope expects radians)
        moduleTable.getEntry("angle").setDouble(state.angle.getRadians());
    }
}