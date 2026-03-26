package frc.robot;

import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FieldDriveSubsystem;
import frc.robot.commands.JuggleCommand;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.ShootGateCommand;
import frc.robot.commands.ClimbArmCommand;
import frc.robot.commands.ClimbMotorCommand;
import frc.robot.commands.ClimbStableCommand;
import frc.robot.commands.FieldDriveCommand;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public class RobotContainer {

    private final FieldDriveSubsystem fieldDriveSystem = new FieldDriveSubsystem();

    private final ShooterSubsystem shooter = new ShooterSubsystem();
    private final ClimbSubsystem climb = new ClimbSubsystem();

    private final XboxController driverController = new XboxController(0);
    private final XboxController operatorController = new XboxController(1);

    public final Pose2d goalPose = new Pose2d(5.0, 4.03, new Rotation2d());

    private static final double JOYSTICK_DEADZONE = 0.075;

    public RobotContainer() {
        configureDefaultCommands();
        configureButtonBindings();
    }

    private void configureDefaultCommands() {

        // Always field-oriented driving
        fieldDriveSystem.setDefaultCommand(
                new FieldDriveCommand(
                        fieldDriveSystem,
                        () -> applyDeadzone(driverController.getLeftY(), JOYSTICK_DEADZONE),
                        () -> applyDeadzone(driverController.getLeftX(), JOYSTICK_DEADZONE),
                        () -> applyDeadzone(-driverController.getRightX(), JOYSTICK_DEADZONE)));
    }

    private void configureButtonBindings() {

        // Spin Up (Right Trigger)
        Trigger rightTrigger = new Trigger(() -> operatorController.getRightTriggerAxis() > 0.2);

        rightTrigger.whileTrue(new ShootCommand(shooter));

        // Juggle (A Button)
        new JoystickButton(operatorController, XboxController.Button.kA.value)
                .whileTrue(new JuggleCommand());

        // Shoot (Left Trigger)
        Trigger leftTrigger = new Trigger(() -> operatorController.getLeftTriggerAxis() > 0.2);

        leftTrigger.whileTrue(new ShootGateCommand(shooter));

        // Climb Arm Toggle (X Button)
        new JoystickButton(operatorController, XboxController.Button.kX.value)
        .onTrue(new ClimbArmCommand(climb));

        // Climb Motor Toggle (Y Button)
        new JoystickButton(operatorController, XboxController.Button.kY.value)
        .onTrue(new ClimbMotorCommand(climb));

        // Climb Bumper Toggle (B Button)
        new JoystickButton(operatorController, XboxController.Button.kB.value)
        .onTrue(new ClimbStableCommand(climb));

    }

    private static double applyDeadzone(double value, double deadzone) {
        if (Math.abs(value) < deadzone)
            return 0.0;

        return Math.signum(value) *
                (Math.abs(value) - deadzone) /
                (1.0 - deadzone);
    }

    public FieldDriveSubsystem getDriveSubsystem() {
        return fieldDriveSystem;
    }

    public Command getAutonomousCommand() {
        return Autos.simpleForwardAuto(fieldDriveSystem);
    }
}