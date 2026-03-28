package frc.robot;

import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;

public final class Autos {

    public static Command simpleBackwardAuto(FieldDriveSubsystem drive, ShooterSubsystem shooter, ClimbSubsystem climb) {

        // Juggle sequence (optional, comment out if testing climb first)
        SequentialCommandGroup juggleSequence = new SequentialCommandGroup(
            new JuggleCommand(), new WaitCommand(0.2),
            new JuggleCommand(), new WaitCommand(0.2),
            new JuggleCommand(), new WaitCommand(0.2),
            new JuggleCommand(), new WaitCommand(0.2),
            new JuggleCommand()
        );


        return new SequentialCommandGroup(

            // Drive backward
            new RunCommand(() -> drive.drive(-0.5, 0.0, 0.0), drive)
                    .withTimeout(1.64),

            // Stop drivetrain
            new InstantCommand(() -> drive.drive(0.0, 0.0, 0.0), drive),

            // Shooter and juggle (optional)
            // new ParallelCommandGroup(
            //     new SequentialCommandGroup(
            //         new InstantCommand(() -> shooter.startFlywheel(0.87), shooter),
            //         new WaitCommand(1.0),
            //         new InstantCommand(shooter::openGate, shooter),
            //         new WaitCommand(3.0),
            //         new InstantCommand(shooter::closeGate, shooter),
            //         new InstantCommand(shooter::stopFlywheel, shooter)
            //     ),
            //     juggleSequence
            // // ),

            new WaitCommand(0.5), // small pause before climb

            // Extend climb cylinders first
            new InstantCommand(climb::extendLongCyls, climb),
            new WaitCommand(2.5), // give pneumatics time to extend

            // Run climb motor to extend (up) for 2 seconds or until top limit hit
            new RunCommand(() -> climb.setMotorPower(-1.5), climb)
                    .withTimeout(3.0),

            // Stop climb motor explicitly (redundant safety)
            new InstantCommand(climb::stopMotor, climb),

            // Retract climb cylinders
            new InstantCommand(climb::retractLongCyls, climb)

        );
    }
}