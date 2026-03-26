package frc.robot.commands;

import frc.robot.subsystems.FieldDriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import java.util.function.Supplier;

public class FieldDriveCommand extends Command {

    private final FieldDriveSubsystem drive;
    private final Supplier<Double> xSup;
    private final Supplier<Double> ySup;
    private final Supplier<Double> rotSup;

    public FieldDriveCommand(FieldDriveSubsystem drive,
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