package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * ClimbRoutineCommand
 *
 * Executes the full climb sequence:
 * long up → motor on → stable out → long down → short up → short down
 * → motor reverse → repeat sequence 2 more times
 *
 * Motor stops after each forward/reverse run, cylinders stay in last positions.
 */
public class ClimbRoutineCommand extends SequentialCommandGroup {

    public ClimbRoutineCommand(ClimbSubsystem climb) {

        // -------------------
        // Timing constants
        // -------------------
        double MOTOR_TIME = 1.0; // seconds motor runs forward or reverse
        double cylinderWait = 3.0;

        addCommands(
                // ----- First cycle -----
                new InstantCommand(climb::extendLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(() -> climb.setMotorPower(1.0), climb),
                new WaitCommand(MOTOR_TIME),
                new InstantCommand(climb::stopMotor, climb), // motor off
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::extendStableCyl, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::extendShortCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractShortCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(() -> climb.setMotorPower(-1.0), climb),
                new WaitCommand(MOTOR_TIME),
                new InstantCommand(climb::stopMotor, climb), // motor off
                new WaitCommand(cylinderWait),

                // ----- Second cycle -----
                new InstantCommand(climb::extendLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(() -> climb.setMotorPower(1.0), climb),
                new WaitCommand(MOTOR_TIME),
                new InstantCommand(climb::stopMotor, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::extendShortCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractShortCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(() -> climb.setMotorPower(-1.0), climb),
                new WaitCommand(MOTOR_TIME),
                new InstantCommand(climb::stopMotor, climb),
                new WaitCommand(cylinderWait),

                // ----- Third cycle -----
                new InstantCommand(climb::extendLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(() -> climb.setMotorPower(1.0), climb),
                new WaitCommand(MOTOR_TIME),
                new InstantCommand(climb::stopMotor, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractLongCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::extendShortCyls, climb),
                new WaitCommand(cylinderWait),
                new InstantCommand(climb::retractShortCyls, climb),
                new WaitCommand(cylinderWait)

        // End of sequence — cylinders stay in their last positions
        // Motor is already stopped after each run
        );
    }
}