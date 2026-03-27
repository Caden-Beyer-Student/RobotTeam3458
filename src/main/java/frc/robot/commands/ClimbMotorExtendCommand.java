package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ClimbSubsystem;

public class ClimbMotorExtendCommand extends Command {

    private final ClimbSubsystem climb;
    private static final double MOTOR_POWER = -1.0; // negative = extend

    public ClimbMotorExtendCommand(ClimbSubsystem climb) {
        this.climb = climb;
        addRequirements(climb);
    }

    @Override
    public void initialize() {
        climb.setMotorPower(MOTOR_POWER);
    }

    @Override
    public boolean isFinished() {
        return climb.isTopLimitHit(); // stop only if top reached
    }

    @Override
    public void end(boolean interrupted) {
        climb.stopMotor();
        climb.setExtended(true);
    }
}