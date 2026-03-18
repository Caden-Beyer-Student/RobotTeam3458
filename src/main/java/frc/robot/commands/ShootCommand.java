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
public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private static final double SPIN_UP_TIME = 1.0;     // seconds to spin up flywheel
    private static final double SPIN_DOWN_DELAY = 0.5;  // seconds flywheel continues after closing gate

    private SequentialCommandGroup shutdownSequence;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // 1️⃣ Safe startup: close gate
        shooter.closeGate();

        // 2️⃣ Start flywheel
        shooter.startFlywheel();

        // 3️⃣ Schedule opening gate after spin-up
        new WaitCommand(SPIN_UP_TIME)
                .andThen(new InstantCommand(shooter::openGate))
                .schedule();
    }

    @Override
    public void end(boolean interrupted) {
        // 4️⃣ Shutdown sequence: close gate, keep flywheel spinning, then stop
        shutdownSequence = new SequentialCommandGroup(
                new InstantCommand(shooter::closeGate),
                new WaitCommand(SPIN_DOWN_DELAY),
                new InstantCommand(shooter::stopFlywheel)
        );
        shutdownSequence.schedule();
    }

    @Override
    public boolean isFinished() {
        // Runs while trigger/button is pressed
        return false;
    }
}
