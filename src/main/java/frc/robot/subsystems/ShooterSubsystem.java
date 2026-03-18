package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * ShooterSubsystem
 *
 * - Two shooter motors (CAN Victor SPX)
 * - Gate motor (PWM Victor SPX)
 */
public class ShooterSubsystem extends SubsystemBase {

    private final VictorSPX shooterLeft = new VictorSPX(17);   // CAN ID
    private final VictorSPX shooterRight = new VictorSPX(16);  // CAN ID
    private final VictorSPX gateMotor = new VictorSPX(0);     // PWM port 0

    private static final double SHOOTER_POWER = 1.0;
    private static final double GATE_OPEN_POWER = 0.5;
    private static final double GATE_CLOSE_POWER = -0.5;

    // -------------------
    // Flywheel
    // -------------------
    public void startFlywheel() {
        shooterLeft.set(ControlMode.PercentOutput, SHOOTER_POWER);
        shooterRight.set(ControlMode.PercentOutput, SHOOTER_POWER);
    }

    public void stopFlywheel() {
        shooterLeft.set(ControlMode.PercentOutput, 0.0);
        shooterRight.set(ControlMode.PercentOutput, 0.0);
    }

    // -------------------
    // Gate
    // -------------------
    public void openGate() {
        gateMotor.set(ControlMode.PercentOutput, GATE_OPEN_POWER);
    }

    public void closeGate() {
        gateMotor.set(ControlMode.PercentOutput, GATE_CLOSE_POWER);
    }

    public void stopGate() {
        gateMotor.set(ControlMode.PercentOutput, 0.0);
    }
}
