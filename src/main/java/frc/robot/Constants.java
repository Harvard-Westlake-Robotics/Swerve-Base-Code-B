package frc.robot;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.PIDConstants;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
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
            public static final int intakeSensorPort = 0; // Intake sensor ID
            public static final boolean intakeMotorInverted = false; // Intake motor inverted
            public static final double kS = 0.0; // Intake kS
            public static final double kV = 0.0; // Intake kV
            public static final double kA = 0.0; // Intake kA
            public static final NeutralModeValue intakeNeutralMode = NeutralModeValue.Brake; // Intake neutral mode
            public static final double intakeKP = 0.0; // Intake kP
            public static final double intakeKI = 0.0; // Intake kI
            public static final double intakeKD = 0.0; // Intake kD

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
        private static SendableChooser<String> autoChooser = new SendableChooser<String>();

        public static HolonomicPathFollowerConfig getPathFollowerConfig() {
            return new HolonomicPathFollowerConfig(translationConstants, rotationConstants,
                    Constants.Swerve.maxSpeed,
                    Constants.Swerve.trackWidth / 2.0, replanningConfig);
        }

        /* Constraint for the motion profilied robot angle controller */
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                kMaxAngularSpeedRadiansPerSecond, kMaxAngularSpeedRadiansPerSecondSquared);
    }
}
