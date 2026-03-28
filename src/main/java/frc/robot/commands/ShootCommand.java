package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommand
 *
 * Runs the flywheel while the button is held.
 */
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final double shooterSpeed;

    public ShootCommand(ShooterSubsystem shooter, double speed) {

        this.shooter = shooter;
        this.shooterSpeed = speed;

        addRequirements(shooter);

    }

    @Override
    public void initialize() {

        shooter.startFlywheel(shooterSpeed);

    }

    @Override
    public void end(boolean interrupted) {

        shooter.stopFlywheel();

    }

    @Override
    public boolean isFinished() {

        return false;

    }
}