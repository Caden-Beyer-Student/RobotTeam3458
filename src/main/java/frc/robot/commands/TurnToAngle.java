package frc.robot.commands;

import frc.robot.subsystems.DriveSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.function.Supplier;

public class TurnToAngle extends Command {

    private final DriveSubsystem drive;
    private final Supplier<Rotation2d> targetSupplier;

    // PD gains
    private final double kP = 0.8;
    private final double kD = 0.0;

    // Stop threshold (radians)
    private final double tolerance = Math.toRadians(2);

    private double lastError = 0.0;

    public TurnToAngle(DriveSubsystem drive, Supplier<Rotation2d> targetSupplier) {
        this.drive = drive;
        this.targetSupplier = targetSupplier;
        addRequirements(drive);
    }

    @Override
    public void execute() {
        Rotation2d targetAngle = targetSupplier.get();

        // Shortest-path angle error
        double error = targetAngle.minus(drive.getBotRotation()).getRadians();
        error = Math.atan2(Math.sin(error), Math.cos(error));

        // Stop rotating if within tolerance
        if (Math.abs(error) < tolerance) {
            drive.drive(0.0, 0.0, 0.0);
            return;
        }

        // Derivative term
        double derivative = (error - lastError) / 0.02; // 20ms loop

        // PD output
        double rotSpeed = kP * error;

        rotSpeed = Math.max(-0.4, Math.min(0.4, rotSpeed));

        // Slow down as we approach the target
        if (Math.abs(error) < Math.toRadians(15)) {
            rotSpeed *= Math.abs(error) / Math.toRadians(15);
        }

        // Clamp output to prevent oscillation
        rotSpeed = Math.max(-0.5, Math.min(0.5, rotSpeed));

        // Rotate only (no translation)
        drive.drive(0.0, 0.0, rotSpeed);

        lastError = error;
    }

    @Override
    public boolean isFinished() {
        Rotation2d targetAngle = targetSupplier.get();
        double error = targetAngle.minus(drive.getBotRotation()).getRadians();
        error = Math.atan2(Math.sin(error), Math.cos(error));
        return Math.abs(error) < tolerance;
    }

    @Override
    public void end(boolean interrupted) {
        // Stop rotation when finished or interrupted
        drive.drive(0.0, 0.0, 0.0);
    }
}