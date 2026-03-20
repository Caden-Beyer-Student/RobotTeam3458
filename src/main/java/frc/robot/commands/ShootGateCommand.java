package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ShooterSubsystem;

/**
 * ShootCommand
 *
 * Sequence:
 * 1. Close gate (safe startup, PWM Victor SPX)
 * 2. Spin up flywheel (CAN Victor SPX motors)
 * 3. Wait SPIN_UP_TIME
 * 4. Open gate to feed balls
 * 5. On release: close gate and keep flywheel SPIN_DOWN_DELAY before stopping
 */
public class ShootGateCommand extends Command {

    ShooterSubsystem shooter;

    public ShootGateCommand(ShooterSubsystem shooter){
        this.shooter = shooter;
    }

    public void initialize() {
        shooter.openGate();

    }

    public void end() {
        shooter.closeGate();

    }

}
