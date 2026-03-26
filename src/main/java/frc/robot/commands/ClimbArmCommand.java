package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbArmCommand extends InstantCommand {

    private static boolean extended = false; // tracks toggle state

    public ClimbArmCommand(ClimbSubsystem climb) {
        super(() -> {
            if (extended) {
                climb.retractLongCyls();
            } else {
                climb.extendLongCyls();
            }
            extended = !extended;
        }, climb);
    }
}