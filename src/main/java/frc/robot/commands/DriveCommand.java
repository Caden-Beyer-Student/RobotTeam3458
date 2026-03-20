package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class DriveCommand extends Command {

    private final DriveSubsystem drive;
    private final Supplier<Double> xSup;
    private final Supplier<Double> ySup;
    private final Supplier<Double> rotSup;

    public DriveCommand(DriveSubsystem drive,
                        Supplier<Double> xSup,
                        Supplier<Double> ySup,
                        Supplier<Double> rotSup) {

        this.drive = drive;
        this.xSup = xSup;
        this.ySup = ySup;
        this.rotSup = rotSup;

        addRequirements(drive);
    }

    @Override
    public void execute() {
        drive.drive(
            xSup.get(),
            ySup.get(),
            rotSup.get()
        );
    }
}