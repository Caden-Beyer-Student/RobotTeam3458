package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;

/**
 * ClimbSubsystem
 *
 * Controls:
 * - Two double-acting cylinders: longCylinders and shortCylinders
 * - One motor (non-speed-controlled)
 *
 * Using REV Pneumatics Hub (REVPH)
 */
public class ClimbSubsystem extends SubsystemBase {

    // -------------------
    // Pneumatics
    // -------------------
    private final DoubleSolenoid longCylinders = new DoubleSolenoid(
            0, // REV Hub CAN ID
            PneumaticsModuleType.REVPH, // module type
            0, // forward channel
            1  // reverse channel
    );

    private final DoubleSolenoid shortCylinders = new DoubleSolenoid(
            0,
            PneumaticsModuleType.REVPH,
            2,
            3
    );

    private final DoubleSolenoid stableCylinder = new DoubleSolenoid(
            0,
            PneumaticsModuleType.REVPH,
            4,
            5
    );

    // -------------------
    // Motor
    // -------------------
    private final PWMSparkMax motor = new PWMSparkMax(5);

    // -------------------
    // Constructor
    // -------------------
    public ClimbSubsystem() {
        // Safe defaults
        longCylinders.set(Value.kReverse); // retracted
        shortCylinders.set(Value.kReverse);
        stableCylinder.set(Value.kReverse);
        motor.set(0.0);                     // stopped
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
        motor.set(power);
    }

    public void stopMotor() {
        motor.set(0.0);
    }

    // -------------------
    // Periodic
    // -------------------
    @Override
    public void periodic() {
        // Optional: telemetry for motor power or solenoid states
        // Example: SmartDashboard.putBoolean("Long Cylinders", longCylinders.get() == Value.kForward);
    }
}