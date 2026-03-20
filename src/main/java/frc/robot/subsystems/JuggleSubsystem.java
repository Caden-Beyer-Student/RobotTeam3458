package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid;

public class JuggleSubsystem extends SubsystemBase {
    public static final DoubleSolenoid m_doubleSolenoid = new DoubleSolenoid(9, PneumaticsModuleType.REVPH, 8, 9);

    public static void solenoidUp() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kForward);

    }

    public static void solenoidDown() {
        m_doubleSolenoid.set(DoubleSolenoid.Value.kReverse);

    }

}
