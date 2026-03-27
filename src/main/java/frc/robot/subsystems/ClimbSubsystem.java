package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * ClimbSubsystem
 *
 * Controls:
 * - Two double-acting cylinders
 * - One motor (CAN Victor SPX)
 * - Two limit switches (DIO)
 */
public class ClimbSubsystem extends SubsystemBase {

    private final DoubleSolenoid longCylinders = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 11, 10);
    private final DoubleSolenoid stableCylinder = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 15, 7);

    private final VictorSPX motor = new VictorSPX(15);

    private final DigitalInput topLimit = new DigitalInput(0);
    private final DigitalInput bottomLimit = new DigitalInput(1);

    private boolean extended = false; // tracks motor state

    public ClimbSubsystem() {
        longCylinders.set(Value.kReverse);
        stableCylinder.set(Value.kReverse);
        motor.set(ControlMode.PercentOutput, 0.0);
        motor.setNeutralMode(NeutralMode.Brake);
    }

    // -------------------
    // Pneumatics
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
    // Motor
    // -------------------
    public void setMotorPower(double power) {
        power = Math.max(-1.0, Math.min(1.0, power)); // clamp power

        if (power > 0) { // moving **retract**
            if (isBottomLimitHit())
                power = 0; // only care about bottom limit
        } else if (power < 0) { // moving **extend**
            if (isTopLimitHit())
                power = 0; // only care about top limit
        }

        motor.set(ControlMode.PercentOutput, power);
    }

    public void stopMotor() {
        motor.set(ControlMode.PercentOutput, 0.0);
    }

    // -------------------
    // Limit Switches
    // -------------------
    public boolean isTopLimitHit() {
        return topLimit.get();
    } // normally open

    public boolean isBottomLimitHit() {
        return bottomLimit.get();
    }

    // -------------------
    // Extended state
    // -------------------
    public boolean isExtended() {
        return extended;
    }

    public void setExtended(boolean state) {
        extended = state;
    }

    @Override
    public void periodic() {
        SmartDashboard.putBoolean("Climb Top Limit", isTopLimitHit());
        SmartDashboard.putBoolean("Climb Bottom Limit", isBottomLimitHit());
        SmartDashboard.putBoolean("Climb Extended", extended);

        if (isTopLimitHit())
            extended = true;
        if (isBottomLimitHit())
            extended = false;
    }
}