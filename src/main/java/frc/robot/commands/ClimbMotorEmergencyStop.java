package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * EmergencyStopCommand
 *
 * Immediately stops the climb motor.
 */
public class ClimbMotorEmergencyStop extends InstantCommand {

    public ClimbMotorEmergencyStop(ClimbSubsystem climb) {
        super(climb::stopMotor, climb); // call stopMotor immediately
    }
}