package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
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
    private final Servo gateServo = new Servo(0);
 
   private final double SHOOTER_POWER = 1.0;
  


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
        //gateServo.setAngle(15);
        gateServo.set(0);
    }

    public void closeGate() {
        //gateServo.setAngle(55);
        gateServo.set(0.5);
    }



}
