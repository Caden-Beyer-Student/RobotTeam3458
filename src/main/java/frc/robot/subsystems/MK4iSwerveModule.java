package frc.robot.subsystems;

// ================= REV =================
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.FeedbackSensor;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.ClosedLoopSlot; // REQUIRED for new setSetpoint API

// ================= CTRE =================
import com.ctre.phoenix6.hardware.CANcoder;

// ================= WPILib =================
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// ================= Units (Phoenix 6 Angle handling) =================
import static edu.wpi.first.units.Units.Rotation; // REQUIRED: Phoenix 6 Angle → rotations

public class MK4iSwerveModule {

    // ================= Hardware =================
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController steerPID;

    private final CANcoder absoluteEncoder;

    // ================= Constants (SDS MK4i L2) =================
    private static final double WHEEL_DIAMETER = 0.1016; // meters
    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final double MAX_SPEED = 4.5; // m/s

    // ================= Feedforward =================
    private static final SimpleMotorFeedforward DRIVE_FF =
            new SimpleMotorFeedforward(0.2, 2.2);

    public MK4iSwerveModule(
            int driveID,
            int steerID,
            int canCoderID,
            double angleOffsetRad
    ) {

        // ---------- Motors ----------
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        drivePID = driveMotor.getClosedLoopController();
        steerPID = steerMotor.getClosedLoopController();

        // ---------- CANcoder ----------
        absoluteEncoder = new CANcoder(canCoderID);

        // ---------- Config Objects ----------
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        // ===== STEER CONFIG =====
        steerConfig
            .inverted(true)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

        steerConfig.encoder
            .positionConversionFactor(
                (2.0 * Math.PI) / STEER_GEAR_RATIO);

        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.0)
            .i(0.0)
            .d(0.1)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0.0, 2.0 * Math.PI);

        // ===== DRIVE CONFIG =====
        driveConfig
            .inverted(false)
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        driveConfig.encoder
            .positionConversionFactor(
                (Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO)
            .velocityConversionFactor(
                ((Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO) / 60.0);

        driveConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(0.1)
            .i(0.0)
            .d(0.0);

        // ---------- Apply Configs ----------
        // CHANGED: use non-deprecated reset/persist enums
        steerMotor.configure(
            steerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // CHANGED: use non-deprecated reset/persist enums
        driveMotor.configure(
            driveConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // ---------- Sync Absolute Encoder ----------
        resetToAbsolute(angleOffsetRad);
    }

    // ================= Control =================
    public void setDesiredState(SwerveModuleState state) {

        // CHANGED: non-deprecated instance optimize
        state.optimize(getAngle());

        // ----- STEER -----
        // CHANGED: correct setSetpoint signature (slot required)
        steerPID.setSetpoint(
            state.angle.getRadians(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        // ----- DRIVE -----
        double velocity = state.speedMetersPerSecond;
        double ffVolts = DRIVE_FF.calculate(velocity);

        // CHANGED: feedforward voltage is NOT a parameter of setSetpoint
        drivePID.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0
        );

        // CHANGED: apply feedforward separately
        driveMotor.setVoltage(ffVolts);
    }

    // ================= State =================
    public Rotation2d getAngle() {
        return Rotation2d.fromRadians(
            steerEncoder.getPosition()
        );
    }

    public double getVelocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    // ================= Absolute Sync =================
    public void resetToAbsolute(double offsetRad) {

        // CHANGED: Phoenix 6 returns StatusSignal<Angle>, NOT double
        // Angle must be converted using units
        double absRotations =
            absoluteEncoder
                .getAbsolutePosition()
                .getValue()
                .in(Rotation);

        double absRadians = absRotations * 2.0 * Math.PI;

        steerEncoder.setPosition(absRadians - offsetRad);
    }
}