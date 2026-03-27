package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbMotorRetractCommand extends Command {

    private final ClimbSubsystem climb;
    private static final double MOTOR_POWER = 1.0; // positive = retract

    public ClimbMotorRetractCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setMotorPower(MOTOR_POWER);
    }

    @Override
    public void execute() {
        // Motor already running, subsystem handles limit switches
    }

    @Override
    public boolean isFinished() {
        if (climb.isBottomLimitHit()) {
            climb.setExtended(false); // now retracted
            return true;
        }
        return false;
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotor();
    }
}