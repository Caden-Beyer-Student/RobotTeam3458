package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public final class Autos {

    public static Command simpleBackwardAuto(FieldDriveSubsystem drive) {

        return new SequentialCommandGroup(
            // Move backward at 50% speed
            new RunCommand(() -> drive.drive(-0.5, 0.0, 0.0), drive)
                .withTimeout(0.5),

            // Stop the robot after moving
            new RunCommand(() -> drive.drive(0.0, 0.0, 0.0), drive)
        );
    }
}