package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbStableCommand extends InstantCommand {

    private static boolean extended = true; // tracks toggle state

    public ClimbStableCommand(ClimbSubsystem climb) {
        super(() -> {
            if (!extended) {
                climb.retractStableCyl();
            } else {
                climb.extendStableCyl();
            }
            extended = !extended;
        }, climb);
    }
}