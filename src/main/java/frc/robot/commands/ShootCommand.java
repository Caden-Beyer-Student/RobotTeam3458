package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Timer;

/**
 * ShootCommand
 *
 * Sequence:
 * 1. Close gate (safe startup)
 * 2. Spin up flywheel
 * 3. Wait 1s
 * 4. Open gate to feed balls
 * 5. On release: close gate and keep flywheel spinning 0.5s before stopping
 */

public class ShootCommand extends Command {

    private final ShooterSubsystem shooter;
    private final Timer timer = new Timer();

    private static final double SPIN_UP_TIME = 1.0;    // seconds to spin up flywheel
    private static final double SPIN_DOWN_DELAY = 0.5; // seconds to keep flywheel on after closing gate

    private boolean gateOpened = false;
    private boolean shuttingDown = false;

    public ShootCommand(ShooterSubsystem shooter) {
        this.shooter = shooter;
        addRequirements(shooter);
    }

    @Override
    public void initialize() {
        // 1️⃣ Safe startup
        shooter.closeGate();

        // 2️⃣ Start flywheel
        shooter.startFlywheel();

        // 3️⃣ Start spin-up timer
        timer.reset();
        timer.start();

        gateOpened = false;
        shuttingDown = false;
    }

    @Override
    public void execute() {
        if (shuttingDown) {
            // 5️⃣ Shutdown sequence: wait SPIN_DOWN_DELAY before stopping flywheel
            if (timer.get() >= SPIN_DOWN_DELAY) {
                shooter.stopFlywheel();
                shuttingDown = false;
            }
        } else {
            // 4️⃣ During shooting: open gate after spin-up
            if (!gateOpened && timer.get() >= SPIN_UP_TIME) {
                shooter.openGate();
                gateOpened = true;
            }
        }
    }

    @Override
    public void end(boolean interrupted) {
        // Start shutdown sequence
        shooter.closeGate();     // close gate immediately
        shuttingDown = true;
        timer.reset();
        timer.start();
    }

    @Override
    public boolean isFinished() {
        // Runs while trigger/button is pressed
        return false;
    }
}