package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;

/**
 * ClimbSubsystem
 *
 * Controls:
 * - Two double-acting cylinders: longCylinders and shortCylinders
 * - One motor (CAN-controlled, open-loop)
 *
 * Using REV Pneumatics Hub (REVPH)
 */
public class ClimbSubsystem extends SubsystemBase {

    // -------------------
    // Pneumatics (REVPH)
    // -------------------
    private final DoubleSolenoid longCylinders = new DoubleSolenoid(
            9, // REVPH CAN ID
            PneumaticsModuleType.REVPH, 
            0, // forward channel
            1  // reverse channel
    );

    private final DoubleSolenoid shortCylinders = new DoubleSolenoid(
            9,
            PneumaticsModuleType.REVPH,
            2,
            3
    );

    private final DoubleSolenoid stableCylinder = new DoubleSolenoid(
            9,
            PneumaticsModuleType.REVPH,
            4,
            5
    );

    // -------------------
    // Motor (CAN Victor SPX)
    // -------------------
    private final VictorSPX motor = new VictorSPX(15); // 15 = CAN ID

    // -------------------
    // Constructor
    // -------------------
    public ClimbSubsystem() {
        // Safe defaults
        longCylinders.set(Value.kReverse);
        shortCylinders.set(Value.kReverse);
        stableCylinder.set(Value.kReverse);
        motor.set(ControlMode.PercentOutput, 0.0); // stopped
    }

    // -------------------
    // Pneumatic methods
    // -------------------
    public void extendLongCyls() {
        longCylinders.set(Value.kForward);
    }

    public void retractLongCyls() {
        longCylinders.set(Value.kReverse);
    }

    public void extendShortCyls() {
        shortCylinders.set(Value.kForward);
    }

    public void retractShortCyls() {
        shortCylinders.set(Value.kReverse);
    }

    public void extendStableCyl() {
        stableCylinder.set(Value.kForward);
    }

    public void retractStableCyl() {
        stableCylinder.set(Value.kReverse);
    }

    // -------------------
    // Motor methods
    // -------------------
    public void setMotorPower(double power) {
        motor.set(ControlMode.PercentOutput, power);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    /**
     * Run the motor at a fixed power for a set duration.
     * Non-blocking option: you could wrap this in a command if needed.
     */
    public void runMotorForSeconds(double power, double seconds) {
        motor.set(ControlMode.PercentOutput, power);
        Timer.delay(seconds); // blocks, simple method for timed runs
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    // -------------------
    // Periodic
    // -------------------
    @Override
    public void periodic() {
        // Optional: telemetry for motor power or solenoid states
    }
}
