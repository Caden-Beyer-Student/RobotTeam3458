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
import com.revrobotics.spark.ClosedLoopSlot;

// ================= CTRE =================
import com.ctre.phoenix6.hardware.CANcoder;

// ================= WPILib =================
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;

// ================= Units =================
import static edu.wpi.first.units.Units.Rotation;

public class MK4iSwerveModule {

    // ================= Hardware =================
    private final SparkMax driveMotor;
    private final SparkMax steerMotor;

    private final SparkClosedLoopController drivePID;
    private final SparkClosedLoopController steerPID;

    private final CANcoder absoluteEncoder;
    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder steerEncoder;

    // ================= Constants =================
    private static final double WHEEL_DIAMETER = 0.1016;
    private static final double DRIVE_GEAR_RATIO = 6.75;
    private static final double STEER_GEAR_RATIO = 150.0 / 7.0;

    private static final SimpleMotorFeedforward DRIVE_FF =
            new SimpleMotorFeedforward(0.2, 2.2);

    private final double angleOffsetRad;

    public MK4iSwerveModule(
            int driveID,
            int steerID,
            int canCoderID,
            double angleOffsetRad
    ) {
        this.angleOffsetRad = angleOffsetRad;

        // ---------- Motors ----------
        driveMotor = new SparkMax(driveID, MotorType.kBrushless);
        steerMotor = new SparkMax(steerID, MotorType.kBrushless);

        drivePID = driveMotor.getClosedLoopController();
        steerPID = steerMotor.getClosedLoopController();

        driveEncoder = driveMotor.getEncoder();
        steerEncoder = steerMotor.getEncoder();

        // ---------- CANcoder ----------
        absoluteEncoder = new CANcoder(canCoderID);

        // FIX: increase CANcoder update frequency so the absolute read at startup is fast
        absoluteEncoder.getAbsolutePosition().setUpdateFrequency(50);

        // ---------- Configs ----------
        SparkMaxConfig driveConfig = new SparkMaxConfig();
        SparkMaxConfig steerConfig = new SparkMaxConfig();

        // ----- Drive -----
        driveConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(40);

        driveConfig.encoder
            .positionConversionFactor(
                (Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO)
            .velocityConversionFactor(
                ((Math.PI * WHEEL_DIAMETER) / DRIVE_GEAR_RATIO) / 60.0);

        driveConfig.closedLoop
            .p(0.6)
            .i(0)
            .d(0);

        // ----- Steer -----
        steerConfig
            .idleMode(IdleMode.kCoast)
            .smartCurrentLimit(20)
            .inverted(true);

        steerConfig.encoder
            .positionConversionFactor(
                (2.0 * Math.PI) / STEER_GEAR_RATIO)
            // FIX: added velocity conversion factor for correct units and telemetry
            .velocityConversionFactor(
                ((2.0 * Math.PI) / STEER_GEAR_RATIO) / 60.0);

        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            // FIX: Increased P gain for better steering response
            .p(0.1)
            .i(0)
            .d(0)
            .positionWrappingEnabled(true)
            .positionWrappingInputRange(0, 2 * Math.PI);

        // ---------- Apply Configs ----------
        driveMotor.configure(
            driveConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        steerMotor.configure(
            steerConfig,
            ResetMode.kNoResetSafeParameters,
            PersistMode.kNoPersistParameters
        );

        // ---------- Sync Steering Encoder ----------
        // FIX: resetToAbsolute now uses stored offset and wraps value to 0-2π
        resetToAbsolute();
    }

    // ================= Control =================
    public void setDesiredState(SwerveModuleState state) {

        // Use module's current angle to prevent unnecessary rotation
        state = SwerveModuleState.optimize(state, getAngle());

        // ----- STEER -----
        // FIX: steering PID now has correct P and wrapped setpoint
        steerPID.setSetpoint(
            state.angle.getRadians(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        // ----- DRIVE -----
        double velocity = state.speedMetersPerSecond;
        double ffVolts = DRIVE_FF.calculate(velocity);

        drivePID.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
    }

    // ================= State =================
    public Rotation2d getAngle() {
        // FIX: Use Spark relative encoder for runtime control
        // The CANcoder is only used once at startup to sync the position
        return Rotation2d.fromRadians(steerEncoder.getPosition());
    }

    public double getVelocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    // ================= Absolute Sync =================
    public void resetToAbsolute() {

        double absRotations =
            absoluteEncoder
                .getAbsolutePosition()
                .getValue()
                .in(Rotation);

        double absRadians = absRotations * 2.0 * Math.PI;

        double corrected = absRadians - angleOffsetRad;

        // FIX: wrap angle into 0-2π for position wrapping
        corrected = (corrected % (2 * Math.PI) + (2 * Math.PI)) % (2 * Math.PI);

        steerEncoder.setPosition(corrected);
    }

    // FIX: optional manual resync method if modules ever desync
    public void syncEncoders() {
        resetToAbsolute();
    }
}