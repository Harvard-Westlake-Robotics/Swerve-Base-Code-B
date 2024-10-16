package frc.robot;

import java.util.List;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.lib.util.COTSTalonFXSwerveConstants;
import frc.lib.util.SwerveModuleConstants;

public final class Constants {
    public static final double stickDeadband = 0.1;

    public static final class Swerve {
        public static final int pigeonID = 18;
        public static final String shooterLimeLightID = "limelight-a";
        public static final String intakeLimeLightID = "limelight-b";

        public static final COTSTalonFXSwerveConstants chosenModule = // TODO: This must be tuned to specific
                                                                      // robot
                COTSTalonFXSwerveConstants.SDS.MK4i
                        .Falcon500(COTSTalonFXSwerveConstants.SDS.MK4i.driveRatios.L2);

        /* Drivetrain Constants */
        public static final double trackWidth = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                             // specific
                                                                             // robot
        public static final double wheelBase = Units.inchesToMeters(21.25); // TODO: This must be tuned to
                                                                            // specific
                                                                            // robot
        public static final double wheelCircumference = chosenModule.wheelCircumference;

        /*
         * Swerve Kinematics
         * No need to ever change this unless you are not doing a traditional
         * rectangular/square 4 module swerve
         */
        public static final SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
                new Translation2d(wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(wheelBase / 2.0, -trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, trackWidth / 2.0),
                new Translation2d(-wheelBase / 2.0, -trackWidth / 2.0));

        /* Module Gear Ratios */
        public static final double driveGearRatio = chosenModule.driveGearRatio;
        public static final double angleGearRatio = chosenModule.angleGearRatio;

        /* Motor Inverts */
        public static final InvertedValue angleMotorInvert = chosenModule.angleMotorInvert;
        public static final InvertedValue driveMotorInvert = chosenModule.driveMotorInvert;

        /* Angle Encoder Invert */
        public static final SensorDirectionValue cancoderInvert = chosenModule.cancoderInvert;

        /* Swerve Current Limiting */
        public static final int angleCurrentLimit = 35;
        public static final int angleCurrentThreshold = 80;
        public static final double angleCurrentThresholdTime = 0.1;
        public static final boolean angleEnableCurrentLimit = true;

        public static final int driveCurrentLimit = 35;
        public static final int driveCurrentThreshold = 80;
        public static final double driveCurrentThresholdTime = 0.1;
        public static final boolean driveEnableCurrentLimit = true;

        /*
         * These values are used by the drive falcon to ramp in open loop and closed
         * loop driving.
         * We found a small open loop ramp (0.25) helps with tread wear, tipping, etc
         */
        public static final double openLoopRamp = 0.25;
        public static final double closedLoopRamp = 0.0;

        /* Angle Motor PID Values */
        public static final double angleKP = chosenModule.angleKP;
        public static final double angleKI = chosenModule.angleKI;
        public static final double angleKD = chosenModule.angleKD;

        /* Drive Motor PID Values */
        public static final double driveKP = 0.120; // TODO: This must be tuned to specific robot
        public static final double driveKI = 0.0;
        public static final double driveKD = 0.00;
        public static final double driveKF = 0.0;

        /* Drive Motor Characterization Values From SYSID */
        public static final double driveKS = 0.32; // TODO: This must be tuned to specific robot
        public static final double driveKV = 1.51;
        public static final double driveKA = 0.27;

        /* Swerve Profiling Values */
        /** Meters per Second */
        public static final double maxSpeed = 4.3; // TODO: This must be tuned to specific robot
        /** Radians per Second */
        public static final double maxAngularVelocity = 10.0; // TODO: This must be tuned to specific robot

        /* Neutral Modes */
        public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Coast;
        public static final NeutralModeValue driveNeutralMode = NeutralModeValue.Coast;

        /* Module Specific Constants */
        /* Front Left Module - Module 0 */
        public static final class Mod0 { // Front Left Module
            public static final int driveMotorID = 7; // Left Front Go motor ID
            public static final int angleMotorID = 8; // Left Front Turn motor ID
            public static final int canCoderID = 23; // Left Front Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(3.24 - 180 - 90); // Adjusted to
            // match the
            // left
            // front encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod1 { // Front Right Module
            public static final int driveMotorID = 1; // Right Front Go motor ID
            public static final int angleMotorID = 2; // Right Front Turn motor ID
            public static final int canCoderID = 20; // Right Front Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(22.68 - 180 - 90); // Adjusted to
            // match the
            // right front
            // encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod2 { // Back Left Module
            public static final int driveMotorID = 5; // Left Back Go motor ID
            public static final int angleMotorID = 6; // Left Back Turn motor ID
            public static final int canCoderID = 22; // Left Back Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(136.8 - 90); // Adjusted to
            // match the
            // left back
            // encoder offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

        public static final class Mod3 { // Back Right Module
            public static final int driveMotorID = 3; // Right Back Go motor ID
            public static final int angleMotorID = 4; // Right Back Turn motor ID
            public static final int canCoderID = 21; // Right Back Encoder CAN ID, assuming it acts as the
                                                     // canCoder for
                                                     // this module
            public static final Rotation2d angleOffset = Rotation2d.fromDegrees(160.2 - 90); // Adjusted to
            // match the
            // right
            // back encoder
            // offset
            public static final SwerveModuleConstants constants = new SwerveModuleConstants(driveMotorID,
                    angleMotorID,
                    canCoderID, angleOffset);
        }

public static final class Intake { // Intake
            public static final int intakeMotorID = 9; // Intake motor ID
            public static final boolean intakeMotorInverted = false; // Intake motor inverted
            public static final double kS = 0.1; // Intake kS
            public static final double kV = 0.05; // Intake kV
            public static final double kA = 0.01; // Intake kA
=======
            public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake; // Intake neutral mode
            public static final double intakeKP = 0.1; // Intake kP
            public static final double intakeKI = 0.0; // Intake kI
            public static final double intakeKD = 0.01; // Intake kD
            public static final double intakeVelocity = 0.8; // Intake velocity
        }

        public static final class Carriage { // Carriage
            public static final int carriageMotorID = 10; // Carriage motor ID
            public static final int carriageSensorPort = 0; // Carriage sensor ID
            public static final boolean carriageMotorInverted = false; // Carriage motor inverted
            public static final double kS = 0.15; // Carriage kS
            public static final double kV = 0.08; // Carriage kV
            public static final double kA = 0.02; // Carriage kA
            public static final NeutralModeValue carriageNeutralMode = NeutralModeValue.Brake; // Carriage neutral mode
            public static final double carriageKP = 0.15; // Carriage kP
            public static final double carriageKI = 0.0; // Carriage kI
            public static final double carriageKD = 0.02; // Carriage kD

            public static final double intakeVelocity = 0.7; // Carriage intake velocity
            public static final double outtakeVelocity = -0.5; // Carriage outtake velocity
            public static final double prepShotVelocity = 0.3; // Carriage prep shot velocity
            public static final double fireVelocity = 1.0; // Carriage fire velocity
            public static final double intakeSlowVelocity = 0.4; // Carriage intake slow velocity
        }

        public static final class Shooter { // Shooter
            public static final int angleMotor1ID = 11; // Angling motor 1 ID
            public static final boolean angleMotor1Inverted = false; // Angling motor 1 inverted
            public static final int angleMotor2ID = 12; // Angling motor 2 ID
            public static final boolean angleMotor2Inverted = true; // Angling motor 2 inverted
            public static final int fireMotor1ID = 13; // Fire motor 1 ID
            public static final boolean fireMotor1Inverted = false; // Fire motor 1 inverted
            public static final int fireMotor2ID = 14; // Fire motor 2 ID
            public static final boolean fireMotor2Inverted = true; // Fire motor 2 inverted
            public static final double anglekS = 0.2; // Angle kS
            public static final double anglekG = 0.5; // Angle kG
            public static final double anglekV = 0.12; // Angle kV
            public static final NeutralModeValue angleNeutralMode = NeutralModeValue.Brake; // Angle neutral mode
            public static final double angleKP = 0.2; // Angle kP
            public static final double angleKI = 0.0; // Angle kI
            public static final double angleKD = 0.02; // Angle kD

            public static final double shootkS = 0.3; // Shoot kS
            public static final double shootkV = 0.15; // Shoot kV
            public static final double shootkA = 0.03; // Shoot kA
            public static final NeutralModeValue fireNeutralMode = NeutralModeValue.Coast; // Shoot neutral mode
            public static final double shootKP = 0.25; // Shoot kP
            public static final double shootKI = 0.0; // Shoot kI
            public static final double shootKD = 0.03; // Shoot kD

            public static final double shootVelocity = 5000.0; // Shoot velocity (RPM)
            public static final double passVelocity = 2000.0; // Pass velocity (RPM)
            public static final double outtakeVelocity = -1000.0; // Outtake velocity (RPM)
            public static final double ampVelocity = 1500.0; // Amp velocity (RPM)

            public static final double downAngle = 15.0; // Down angle (degrees)
            public static final double upAngle = 75.0; // Up angle (degrees)

            public static final int angleSensorPort = 1; // Angle sensor port
        }
    }

    public static final class AutoConstants { // TODO: The below constants are used in the example auto, and must be
                                              // tuned to specific robot
        public static final double kMaxSpeedMetersPerSecond = 3;
        public static final double kMaxAccelerationMetersPerSecondSquared = 3;
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI;
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI;

        public static final double kPXController = 1;
        public static final double kPYController = 1;
        public static final double kPThetaController = 1;

        private static PIDConstants translationConstants = new PIDConstants(1.1, 0.0, 0.0);
        private static PIDConstants rotationConstants = new PIDConstants(1.9, 0, 0.0);
        private static ReplanningConfig replanningConfig = new ReplanningConfig();
        private static SendableChooser<PathPlannerAuto> autoChooser = new SendableChooser<PathPlannerAuto>();

        public static SendableChooser<PathPlannerAuto> getAutoChooser() {
            return autoChooser;
        }

        public static void configureAutos() {
            autoChooser.setDefaultOption("", new PathPlannerAuto("null"));
        }

        public static HolonomicPathFollowerConfig getPathFollowerConfig() {
            return new HolonomicPathFollowerConfig(translationConstants, rotationConstants,
                    Constants.Swerve.maxSpeed,
                    Constants.Swerve.trackWidth / 2.0, replanningConfig);
        }

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }

public static final class Vision {

    public static final class CameraConfig {
        public final String name;
        public final Transform3d robotToCamera;

        public CameraConfig(String name, Transform3d robotToCamera) {
            this.name = name;
            this.robotToCamera = robotToCamera;
        }
    }

    public static final List<CameraConfig> CAMERAS = List.of(
        new CameraConfig("Camera1", new Transform3d(
            new Translation3d(0.5, 0.0, 0.5), new Rotation3d(0, 0, 0))),
        new CameraConfig("Camera2", new Transform3d(
            new Translation3d(-0.5, 0.0, 0.5), new Rotation3d(0, Math.PI, 0))),
        new CameraConfig("Camera3", new Transform3d(
            new Translation3d(0.0, 0.5, 0.5), new Rotation3d(0, Math.PI / 2, 0))),
        new CameraConfig("Camera4", new Transform3d(
            new Translation3d(0.0, -0.5, 0.5), new Rotation3d(0, -Math.PI / 2, 0))),
        new CameraConfig("Camera5", new Transform3d(
            new Translation3d(0.0, 0.0, 1.0), new Rotation3d(-Math.PI / 2, 0, 0))),
        new CameraConfig("Camera6", new Transform3d(
            new Translation3d(0.0, 0.0, 0.0), new Rotation3d(Math.PI / 2, 0, 0)))
    );

    // The layout of the AprilTags on the field
    public static final AprilTagFieldLayout kTagLayout =
            AprilTagFields.kDefaultField.loadAprilTagLayoutField();

    // The standard deviations of our vision estimated poses, which affect correction rate
    // (Fake values. Experiment and determine estimation noise on an actual robot.)
    public static final Matrix<N3, N1> kSingleTagStdDevs = VecBuilder.fill(4, 4, 8);
    public static final Matrix<N3, N1> kMultiTagStdDevs = VecBuilder.fill(0.5, 0.5, 1);
}
}
