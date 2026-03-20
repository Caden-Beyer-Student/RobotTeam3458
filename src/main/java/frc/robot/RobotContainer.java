package frc.robot;

import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.TurnToAngle;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ClimbRoutineCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotContainer {

    // ========================
    // Subsystems
    // ========================
    private final DriveSubsystem drive = new DriveSubsystem();
    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem(); 
    // ^ Added climb subsystem so the climb command has something to require

    // ========================
    // Controllers
    // ========================
    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    // ========================
    // Goal pose for aiming
    // ========================
    public final Pose2d goalPose = new Pose2d(5.0, 4.03, new Rotation2d());

    private static final double JOYSTICK_DEADZONE = 0.075;

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    // ------------------------
    // Default commands
    // ------------------------
    private void configureDefaultCommands() {
        // Default drive command
        drive.setDefaultCommand(
            new DriveCommand(
                drive,
                () -> applyDeadzone(-driverController.getLeftY(), JOYSTICK_DEADZONE),
                () -> applyDeadzone(-driverController.getLeftX(), JOYSTICK_DEADZONE),
                () -> applyDeadzone(driverController.getRightX(), JOYSTICK_DEADZONE)
            )
        );
    }

    // ------------------------
    // Button bindings
    // ------------------------
    private void configureButtonBindings() {

        // ------------------------
        // Driver controls
        // ------------------------

        // Driver B button: aim at goal
        // Driver B button: aim at goal
new JoystickButton(driverController, XboxController.Button.kB.value)
    .onTrue(new TurnToAngle(drive, () -> drive.getAngleToGoal(goalPose)));        // ------------------------
        // Operator controls
        // ------------------------

        // Operator right trigger: shoot while held
        Trigger rightTrigger =
            new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2);
        rightTrigger.whileTrue(new ShootCommand(shooter));

        // ------------------------
        // CLIMB SAFETY COMBINATION
        // Right Bumper + Left Bumper + A must ALL be held
        // ------------------------

        JoystickButton rightBumper =
            new JoystickButton(operatorController, XboxController.Button.kRightBumper.value);

        JoystickButton leftBumper =
            new JoystickButton(operatorController, XboxController.Button.kLeftBumper.value);

        JoystickButton aButton =
            new JoystickButton(operatorController, XboxController.Button.kA.value);

        Trigger climbTrigger =
            rightBumper
                .and(leftBumper)
                .and(aButton);

        // Run climb routine ONLY while all three buttons are held
        climbTrigger.onTrue(
            new ClimbRoutineCommand(climb)
        );
    }

    // ------------------------
    // Deadzone helper
    // ------------------------
    private static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone)
            return 0.0;
        return Math.signum(value) * (Math.abs(value) - deadzone) / (1.0 - deadzone);
    }

    // ------------------------
    // Getters
    // ------------------------
    public DriveSubsystem getDriveSubsystem() {
        return drive;
    }

    public Command getAutonomousCommand() {
        return Autos.simpleForwardAuto(drive);
    }
}