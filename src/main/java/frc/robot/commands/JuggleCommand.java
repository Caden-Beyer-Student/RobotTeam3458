package frc.robot.commands;

import frc.robot.subsystems.JuggleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class JuggleCommand extends Command {

    public void initialize() {
        JuggleSubsystem.solenoidUp();

    }

    public void end() {
        JuggleSubsystem.solenoidDown();

    }

}