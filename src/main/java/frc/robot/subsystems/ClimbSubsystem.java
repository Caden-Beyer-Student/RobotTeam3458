package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode; // ADDED
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput; // ADDED
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard; // ADDED

/**
 * ClimbSubsystem
 *
 * Controls:
 * - Two double-acting cylinders
 * - One motor (CAN Victor SPX)
 * - Two limit switches (DIO)
 *
 * Using REV Pneumatics Hub (REVPH)
 */
public class ClimbSubsystem extends SubsystemBase {

    // -------------------
    // Pneumatics (REVPH)
    // -------------------
    private final DoubleSolenoid longCylinders = new DoubleSolenoid(
            9,
            PneumaticsModuleType.REVPH,
            11,
            10
    );

    private final DoubleSolenoid stableCylinder = new DoubleSolenoid(
            9,
            PneumaticsModuleType.REVPH,
            7,
            15
    );

    // -------------------
    // Motor (CAN Victor SPX)
    // -------------------
    private final VictorSPX motor = new VictorSPX(15);

    // -------------------
    // LIMIT SWITCHES (ADDED)
    // -------------------
    private final DigitalInput topLimit = new DigitalInput(0);     // DIO 0
    private final DigitalInput bottomLimit = new DigitalInput(1);  // DIO 1

    // -------------------
    // Constructor
    // -------------------
    public ClimbSubsystem() {

        longCylinders.set(Value.kReverse);
        stableCylinder.set(Value.kReverse);

        motor.set(ControlMode.PercentOutput, 0.0);

        // ADDED: ensure motor stops quickly
        motor.setNeutralMode(NeutralMode.Brake);
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

        // ADDED: clamp power for safety
        power = Math.max(-1.0, Math.min(1.0, power));

        // ADDED: software limit protection
        if (power > 0 && isTopLimitHit()) {
            power = 0;
        }

        if (power < 0 && isBottomLimitHit()) {
            power = 0;
        }

        motor.set(ControlMode.PercentOutput, power);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    // -------------------
    // LIMIT SWITCH HELPERS (ADDED)
    // -------------------
    public boolean isTopLimitHit() {
        return topLimit.get(); // normally closed switch
    }

    public boolean isBottomLimitHit() {
        return bottomLimit.get();
    }

    // -------------------
    // Periodic
    // -------------------
    @Override
    public void periodic() {

        // ADDED: telemetry for debugging
        SmartDashboard.putBoolean("Climb Top Limit", isTopLimitHit());
        SmartDashboard.putBoolean("Climb Bottom Limit", isBottomLimitHit());
    }
}