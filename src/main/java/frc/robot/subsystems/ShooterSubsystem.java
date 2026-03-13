package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

// Telemetry
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.DoublePublisher;
import edu.wpi.first.networktables.BooleanPublisher;

/**
 * ShooterSubsystem
 *
 * Owns:
 *  - Two controllable flywheel motors
 *  - One servo gate that blocks/allows input
 *
 * NOTE:
 *  - All simulation-related code (ServoSim, simulationPeriodic)
 *    has been intentionally removed because this subsystem
 *    will NOT be tested in WPILib simulation.
 */
public class ShooterSubsystem extends SubsystemBase {

    // ========================
    // Hardware
    // ========================

    private final PWMSparkMax shooterMotorLeft;
    private final PWMSparkMax shooterMotorRight;
    private final Servo gateServo;

    // ========================
    // Telemetry
    // ========================

    private final DoublePublisher gateAngleDegPub;
    private final DoublePublisher flywheelPowerPub;
    private final BooleanPublisher flywheelRunningPub;

    // ========================
    // Constants
    // ========================

    // Servo angles (tune on real robot)
    private static final double GATE_CLOSED_DEG = 0.0;
    private static final double GATE_OPEN_DEG   = 90.0;

    // Default shooter power (open-loop)
    private static final double DEFAULT_SHOOTER_POWER = 1.0;

    // ========================
    // State
    // ========================

    private double currentFlywheelPower = 0.0;

    // ========================
    // Constructor
    // ========================

    public ShooterSubsystem() {

        // --- Motors ---
        shooterMotorLeft  = new PWMSparkMax(0); // TODO: correct PWM port
        shooterMotorRight = new PWMSparkMax(1); // TODO: correct PWM port

        shooterMotorLeft.setInverted(false);
        shooterMotorRight.setInverted(true); // common for opposing flywheels

        // --- Servo ---
        gateServo = new Servo(2); // TODO: correct PWM port

        // --- Telemetry ---
        var table = NetworkTableInstance.getDefault().getTable("Shooter");

        gateAngleDegPub = table.getDoubleTopic("GateAngleDeg").publish();
        flywheelPowerPub = table.getDoubleTopic("FlywheelPower").publish();
        flywheelRunningPub = table.getBooleanTopic("FlywheelRunning").publish();

        // --- Safe startup state ---
        // COMMENT: Ensures robot boots with shooter inactive
        closeGate();
        stopFlywheel();
    }

    // ========================
    // Flywheel control
    // ========================

    /**
     * Set flywheel power (open-loop).
     * Range: [-1.0, 1.0]
     */
    public void setFlywheelPower(double power) {
        currentFlywheelPower = power;
        shooterMotorLeft.set(power);
        shooterMotorRight.set(power);
    }

    /** Start flywheel at default shooting power */
    public void startFlywheel() {
        setFlywheelPower(DEFAULT_SHOOTER_POWER);
    }

    /** Stop flywheel motors */
    public void stopFlywheel() {
        setFlywheelPower(0.0);
    }

    /** Returns true if flywheel is commanded on */
    public boolean isFlywheelRunning() {
        return Math.abs(currentFlywheelPower) > 0.01;
    }

    /** Returns current commanded flywheel power */
    public double getFlywheelPower() {
        return currentFlywheelPower;
    }

    // ========================
    // Gate (servo) control
    // ========================

    /** Close gate to block game piece */
    public void closeGate() {
        gateServo.setAngle(GATE_CLOSED_DEG);
    }

    /** Open gate to allow shooting */
    public void openGate() {
        gateServo.setAngle(GATE_OPEN_DEG);
    }

    /** Get current gate angle (degrees) */
    public double getGateAngleDeg() {
        return gateServo.getAngle();
    }

    // ========================
    // Periodic
    // ========================

    @Override
    public void periodic() {
        // Telemetry (real robot only)
        gateAngleDegPub.set(getGateAngleDeg());
        flywheelPowerPub.set(getFlywheelPower());
        flywheelRunningPub.set(isFlywheelRunning());
    }

    /*
     * REMOVED:
     *  - simulationPeriodic()
     *  - ServoSim
     *
     * REASON:
     *  - Shooter will not be tested in desktop simulation
     *  - Removing sim code reduces noise and confusion
     *  - No impact on real-robot behavior
     */
}