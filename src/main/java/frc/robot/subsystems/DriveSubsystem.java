package frc.robot.subsystems;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.*;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class DriveSubsystem extends SubsystemBase {

        private static final double MAX_SPEED = 3.0; // m/s
        private static final double MAX_ANGULAR_SPEED = 2 * Math.PI;

        private final double TRACK_WIDTH = 0.6;
        private final double WHEEL_BASE = 0.6;

        private final Field2d field = new Field2d();

        // Initialize MK4iSwerveModules with proper CAN IDs and offsets
        private final MK4iSwerveModule frontLeft = new MK4iSwerveModule(1, 2, 11, 1.570796);
        private final MK4iSwerveModule frontRight = new MK4iSwerveModule(3, 4, 12, 1.570796);
        private final MK4iSwerveModule backLeft = new MK4iSwerveModule(5, 6, 13, 1.570796);
        private final MK4iSwerveModule backRight = new MK4iSwerveModule(7, 8, 14, 1.570796);

        private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(
                        new Translation2d(WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(WHEEL_BASE / 2, -TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, TRACK_WIDTH / 2),
                        new Translation2d(-WHEEL_BASE / 2, -TRACK_WIDTH / 2));

        private Pose2d pose = new Pose2d();
        private Rotation2d robotAngle = new Rotation2d();
        private double lastTime;

        // Stores latest desired module states for telemetry
        private SwerveModuleState[] lastModuleStates = new SwerveModuleState[] {
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState(),
                        new SwerveModuleState()
        };

        public DriveSubsystem() {
                pose = new Pose2d(2.0, 4.0, Rotation2d.fromDegrees(180));
                robotAngle = pose.getRotation();
                lastTime = Timer.getFPGATimestamp();

                SmartDashboard.putData("Field", field);
        }

        public void drive(double xInput, double yInput, double rotInput) {
                // Stick magnitude and angle
                double stickMagnitude = Math.min(1.0, Math.sqrt(xInput * xInput + yInput * yInput));
                double stickAngle = Math.atan2(yInput, xInput);

                // Desired chassis speeds
                double xSpeed = stickMagnitude * MAX_SPEED * Math.cos(stickAngle);
                double ySpeed = stickMagnitude * MAX_SPEED * Math.sin(stickAngle);
                double rotSpeed = rotInput * MAX_ANGULAR_SPEED;

                ChassisSpeeds commandedSpeeds = ChassisSpeeds.fromFieldRelativeSpeeds(
                                xSpeed, ySpeed, rotSpeed, robotAngle);

                // Convert to module states
                SwerveModuleState[] desiredStates = kinematics.toSwerveModuleStates(commandedSpeeds);

                // Desaturate speeds
                SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, MAX_SPEED);

                // Optimize each module using the actual module angle
                // Instead of this (causes void -> SwerveModuleState error):
                // desiredStates[0].optimize(frontLeft.getAngle());

                // Do this:
                desiredStates[0].optimize(frontLeft.getAngle());
                desiredStates[1].optimize(frontRight.getAngle());
                desiredStates[2].optimize(backLeft.getAngle());
                desiredStates[3].optimize(backRight.getAngle());

                // Apply to modules
                frontLeft.setDesiredState(desiredStates[0]);
                frontRight.setDesiredState(desiredStates[1]);
                backLeft.setDesiredState(desiredStates[2]);
                backRight.setDesiredState(desiredStates[3]);

                // Save for telemetry
                lastModuleStates = desiredStates.clone();
        }

        @Override
        public void periodic() {
                double currentTime = Timer.getFPGATimestamp();
                double deltaTime = Math.max(0, currentTime - lastTime);
                lastTime = currentTime;

                // Read actual module states
                SwerveModuleState[] actualStates = new SwerveModuleState[] {
                                new SwerveModuleState(backLeft.getVelocityMetersPerSecond(), backLeft.getAngle()),
                                new SwerveModuleState(backRight.getVelocityMetersPerSecond(), backRight.getAngle()),
                                new SwerveModuleState(frontLeft.getVelocityMetersPerSecond(), frontLeft.getAngle()),
                                new SwerveModuleState(frontRight.getVelocityMetersPerSecond(), frontRight.getAngle())
                };

                // Convert to chassis speeds
                ChassisSpeeds actualChassisSpeeds = kinematics.toChassisSpeeds(actualStates);

                // Integrate pose
                double dx = actualChassisSpeeds.vxMetersPerSecond * deltaTime;
                double dy = actualChassisSpeeds.vyMetersPerSecond * deltaTime;
                double dtheta = actualChassisSpeeds.omegaRadiansPerSecond * deltaTime;

                pose = new Pose2d(
                                pose.getX() + dx * Math.cos(pose.getRotation().getRadians())
                                                - dy * Math.sin(pose.getRotation().getRadians()),
                                pose.getY() + dx * Math.sin(pose.getRotation().getRadians())
                                                + dy * Math.cos(pose.getRotation().getRadians()),
                                pose.getRotation().plus(new Rotation2d(dtheta)));

                robotAngle = pose.getRotation();

                // Publish pose to NetworkTables
                NetworkTableInstance.getDefault()
                                .getTable("Swerve")
                                .getEntry("Pose")
                                .setDoubleArray(new double[] { pose.getX(), pose.getY(), robotAngle.getRadians() });

                // Publish module states for telemetry
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

        // ===== Helper Getters =====
        public Pose2d getPose() {
                return pose;
        }

        public Rotation2d getBotRotation() {
                return robotAngle;
        }

        public void resetPose(Pose2d newPose) {
                pose = newPose;
                robotAngle = newPose.getRotation();
        }

        public Rotation2d getAngleToGoal(Pose2d goal) {
                double dx = goal.getX() - pose.getX();
                double dy = goal.getY() - pose.getY();
                return new Rotation2d(Math.atan2(dy, dx));
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