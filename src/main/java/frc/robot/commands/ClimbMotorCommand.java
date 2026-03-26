package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

/**
 * ClimbMotorCommand
 *
 * Toggles the claw motor direction on each press.
 * Stops automatically when the corresponding limit switch is triggered.
 */
public class ClimbMotorCommand extends Command {

    private final ClimbSubsystem climb;

    // Tracks toggle state: true = extended, false = retracted
    private static boolean extended = false;

    // Motor power
    private static final double MOTOR_POWER = 1;

    public ClimbMotorCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        // Determine direction based on toggle state
        if (extended) {
            // retracting
            climb.setMotorPower(-MOTOR_POWER);
        } else {
            // extending
            climb.setMotorPower(MOTOR_POWER);
        }
    }

    @Override
    public void execute() {
        // motor already running; the subsystem handles limit switches
    }

    @Override
    public boolean isFinished() {
        // Stop if motor hits the corresponding limit switch
        if (!extended && climb.isTopLimitHit()) {
            extended = true;
            return true;
        }
        if (extended && climb.isBottomLimitHit()) {
            extended = false;
            return true;
        }
        return false; // keep running otherwise
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotor();
    }
}