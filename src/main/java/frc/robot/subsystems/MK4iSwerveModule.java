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
            .p(0.1)
            .i(0)
            .d(0);

        // ----- Steer -----
        steerConfig
            .idleMode(IdleMode.kBrake)
            .smartCurrentLimit(20);

        steerConfig.encoder
            .positionConversionFactor(
                (2.0 * Math.PI) / STEER_GEAR_RATIO);

        steerConfig.closedLoop
            .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
            .p(1.0)
            .i(0)
            .d(0.1)
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
        resetToAbsolute(angleOffsetRad);
    }

    // ================= Control =================
    public void setDesiredState(SwerveModuleState state) {

        // FIX: use new WPILib optimize call to avoid unnecessary 180° flips
        state = SwerveModuleState.optimize(state, getAngle());

        // ----- STEER -----
        steerPID.setSetpoint(
            state.angle.getRadians(),
            ControlType.kPosition,
            ClosedLoopSlot.kSlot0
        );

        // ----- DRIVE -----
        double velocity = state.speedMetersPerSecond;
        double ffVolts = DRIVE_FF.calculate(velocity);

        // FIX: combine feedforward with PID rather than overwriting
        drivePID.setSetpoint(
            velocity,
            ControlType.kVelocity,
            ClosedLoopSlot.kSlot0,
            ffVolts
        );
    }

    // ================= State =================
    public Rotation2d getAngle() {
        double absRotations =
            absoluteEncoder
                .getAbsolutePosition()
                .getValue()
                .in(Rotation);

        double radians = absRotations * 2.0 * Math.PI - angleOffsetRad;
        return Rotation2d.fromRadians(radians);
    }

    public double getVelocityMetersPerSecond() {
        return driveEncoder.getVelocity();
    }

    // ================= Absolute Sync =================
    public void resetToAbsolute(double offsetRad) {
        double absRotations =
            absoluteEncoder
                .getAbsolutePosition()
                .getValue()
                .in(Rotation);

        double absRadians = absRotations * 2.0 * Math.PI;

        // Write to Spark encoder for PID control
        steerEncoder.setPosition(absRadians - offsetRad);
    }
}
