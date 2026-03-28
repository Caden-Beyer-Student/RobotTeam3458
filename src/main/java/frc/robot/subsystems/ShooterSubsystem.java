package frc.robot.subsystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;

/**
 * ShooterSubsystem
 *
 * - Two shooter motors (CAN Victor SPX)
 * - Gate solenoid
 */
public class ShooterSubsystem extends SubsystemBase {

    private final VictorSPX shooterLeft = new VictorSPX(17);
    private final VictorSPX shooterRight = new VictorSPX(16);

    public final DoubleSolenoid gateSolenoid =
            new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 12, 13);

    // -------------------
    // Flywheel
    // -------------------

    /**
     * Starts flywheel at specified power
     * @param power Percent output (-1 to 1)
     */
    public void startFlywheel(double power) {

        shooterLeft.set(ControlMode.PercentOutput, power);
        shooterRight.set(ControlMode.PercentOutput, power);

    }

    public void stopFlywheel() {

        shooterLeft.set(ControlMode.PercentOutput, 0.0);
        shooterRight.set(ControlMode.PercentOutput, 0.0);

    }

    // -------------------
    // Gate
    // -------------------

    public void openGate() {

        gateSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    public void closeGate() {

        gateSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

}