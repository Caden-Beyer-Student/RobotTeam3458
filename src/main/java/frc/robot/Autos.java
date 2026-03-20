package frc.robot;

import frc.robot.subsystems.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public final class Autos {

    public static Command simpleForwardAuto(DriveSubsystem drive) {

        return new SequentialCommandGroup(
                new RunCommand(
                        () -> drive.drive(1, 0, 0),
                        drive
                ).withTimeout(1)
        );
    }
}
