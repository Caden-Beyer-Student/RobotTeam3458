package frc.robot.subsystems;

import com.studica.frc.AHRS;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class FieldDriveSubsystem extends SubsystemBase {

    private static final double MAX_SPEED = 3;
    private static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

    private final double TRACK_WIDTH = 19.5 / 39.375;
    private final double WHEEL_BASE = 25 / 39.375;

    private final Field2d field = new Field2d();

    private final MK4iSwerveModule frontLeft = new MK4iSwerveModule(1, 2, 11, 0);
    private final MK4iSwerveModule frontRight = new MK4iSwerveModule(3, 4, 12, 0);
    private final MK4iSwerveModule backLeft = new MK4iSwerveModule(6, 5, 13, 0);
    private final MK4iSwerveModule backRight = new MK4iSwerveModule(7, 8, 14, 0);

    private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
            new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
            new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

    private Pose2d pose = new Pose2d();
    private Rotation2d robotAngle = new Rotation2d();
    private double lastTime;

    private final AHRS m_gyro = new AHRS(AHRS.NavXComType.kMXP_SPI, 50);
    private double initialHeading = 0.0;

    private SwerveModuleState[] lastModuleStates = new SwerveModuleState[] {
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState(),
            new SwerveModuleState()
    };

    public FieldDriveSubsystem() {

        frontLeft.syncEncoders();
        frontRight.syncEncoders();
        backLeft.syncEncoders();
        backRight.syncEncoders();

        pose = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(180));
        robotAngle = pose.getRotation();
        lastTime = Timer.getFPGATimestamp();

        SmartDashboard.putData("Field", field);

        m_gyro.reset();
        initialHeading = m_gyro.getYaw();
    }

    public void drive(double xInput, double yInput, double rotInput) {

        double stickMagnitude = Math.min(1.0, Math.sqrt(xInput * xInput + yInput * yInput));
        double stickAngle = Math.atan2(yInput, xInput);

        double xSpeed = stickMagnitude * MAX_SPEED * Math.cos(stickAngle);
        double ySpeed = stickMagnitude * MAX_SPEED * Math.sin(stickAngle);
        double rotSpeed = rotInput * MAX_ANGULAR_SPEED;

        // FIX: Use live gyro reading for field-oriented control
        Rotation2d heading = Rotation2d.fromDegrees(-(m_gyro.getYaw() - initialHeading));

        ChassisSpeeds commandedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                xSpeed,
                ySpeed,
                rotSpeed,
                heading // live gyro heading instead of robotAngle
        );

        SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(commandedSpeeds);

        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

        frontLeft.setDesiredState(desiredStates[0]);
        frontRight.setDesiredState(desiredStates[1]);
        backLeft.setDesiredState(desiredStates[2]);
        backRight.setDesiredState(desiredStates[3]);

        lastModuleStates = desiredStates.clone();
    }

    @Override
    public void periodic() {

        double currentTime = Timer.getFPGATimestamp();
        double deltaTime = Math.max(0, currentTime - lastTime);
        lastTime = currentTime;

        SmartDashboard.putNumber("FrontLeftEncoder", frontLeft.getAngle().getDegrees());
        SmartDashboard.putNumber("FrontRightEncoder", frontRight.getAngle().getDegrees());
        SmartDashboard.putNumber("BackLeftEncoder", backLeft.getAngle().getDegrees());
        SmartDashboard.putNumber("BackRightEncoder", backRight.getAngle().getDegrees());

        // FIX: Same corrected gyro orientation
        robotAngle = Rotation2d.fromDegrees(-(m_gyro.getYaw() - initialHeading));

        SmartDashboard.putNumber("Gyro Yaw", m_gyro.getYaw());

        field.setRobotPose(pose);

        NetworkTableInstance.getDefault()
                .getTable("Swerve")
                .getEntry("Pose")
                .setDoubleArray(new double[] {
                        pose.getX(),
                        pose.getY(),
                        robotAngle.getRadians()
                });

        double[] flatStates = new double[lastModuleStates.length * 2];

        for (int i = 0; i < lastModuleStates.length; i++) {
            flatStates[i * 2] = lastModuleStates[i].angle.getRadians();
            flatStates[i * 2 + 1] = lastModuleStates[i].speedMetersPerSecond;
        }

        NetworkTableInstance.getDefault()
                .getTable("Swerve")
                .getEntry("ModuleStates")
                .setDoubleArray(flatStates);
    }

    public Pose2d getPose() {
        return pose;
    }

    public Rotation2d getBotRotation() {
        return robotAngle;
    }

    public void resetPose(Pose2d newPose) {
        pose = newPose;
        robotAngle = newPose.getRotation();
        initialHeading = m_gyro.getYaw();
    }

    public Rotation2d getAngleToGoal(Pose2d goal) {
        double dx = goal.getX() - pose.getX();
        double dy = goal.getY() - pose.getY();
        return new Rotation2d(Math.atan2(dy, dx));
    }

    public double getHeadingError(double targetDegrees) {
        double current = -(m_gyro.getYaw() - initialHeading);
        double error = targetDegrees - current;
        return ((error + 180) % 360) - 180;
    }

    public MK4iSwerveModule getFrontLeft() {
        return frontLeft;
    }

    public MK4iSwerveModule getFrontRight() {
        return frontRight;
    }

    public MK4iSwerveModule getBackLeft() {
        return backLeft;
    }

    public MK4iSwerveModule getBackRight() {
        return backRight;
    }
}