package frc.robot.subsystems;

import java.util.Vector;

import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private TalonFX angleMotor1;
    private TalonFX angleMotor2;
    private double velocity = 0.0;
    private double angle = 0.0;
    private SendableChooser<NeutralModeValue> angleNeutralModeChooser = new SendableChooser<>();
    private SendableChooser<NeutralModeValue> fireNeutralModeChooser = new SendableChooser<>();
    private MotionMagicVelocityTorqueCurrentFOC fireControl;
    private MotionMagicTorqueCurrentFOC angleControl;
    private NeutralModeValue currentFireNeutralMode = Constants.Swerve.Shooter.fireNeutralMode;
    private NeutralModeValue currentAngleNeutralMode = Constants.Swerve.Shooter.angleNeutralMode;

    public static Shooter getInstance() {
        if (instance == null) {
            instance = new Shooter();
        }
        return instance;
    }

    private Shooter() {
        this.shooterMotor1 = new TalonFX(Constants.Swerve.Shooter.fireMotor1ID);
        this.shooterMotor2 = new TalonFX(Constants.Swerve.Shooter.fireMotor2ID);
        this.angleMotor1 = new TalonFX(Constants.Swerve.Shooter.angleMotor1ID);
        this.angleMotor2 = new TalonFX(Constants.Swerve.Shooter.angleMotor2ID);
        this.shooterMotor1.setInverted(Constants.Swerve.Shooter.fireMotor1Inverted);
        this.shooterMotor2.setInverted(Constants.Swerve.Shooter.fireMotor2Inverted);
        this.angleMotor1.setInverted(Constants.Swerve.Shooter.angleMotor1Inverted);
        this.angleMotor2.setInverted(Constants.Swerve.Shooter.angleMotor2Inverted);
        this.shooterMotor1.setNeutralMode(Constants.Swerve.Shooter.fireNeutralMode);
        this.shooterMotor2.setNeutralMode(Constants.Swerve.Shooter.fireNeutralMode);
        this.angleMotor1.setNeutralMode(Constants.Swerve.Shooter.angleNeutralMode);
        this.angleMotor2.setNeutralMode(Constants.Swerve.Shooter.angleNeutralMode);
        this.fireControl = new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0, 0, false, true, false);
        this.angleControl = new MotionMagicTorqueCurrentFOC(angle, 0.0, 0, false, false, false);
        this.shooterMotor1.setControl(fireControl);
        this.shooterMotor2.setControl(fireControl);
        this.angleMotor1.setControl(angleControl);
        this.angleMotor2.setControl(angleControl);

        fireNeutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        fireNeutralModeChooser.addOption("Coast", NeutralModeValue.Coast);
        angleNeutralModeChooser.setDefaultOption("Brake", NeutralModeValue.Brake);
        angleNeutralModeChooser.addOption("Coast", NeutralModeValue.Coast);

    }

    public void setVelocity(double speed) {
        velocity = speed;
    }

    public void fire() {
        velocity = Constants.Swerve.Shooter.shootVelocity;
    }

    public void outtake() {
        velocity = Constants.Swerve.Shooter.outtakeVelocity;
    }

    public void amp() {
        velocity = Constants.Swerve.Shooter.ampVelocity;
    }

    public void setAngle(double angle) {
        this.angle = angle;
    }

    public void stop() {
        velocity = 0.0;
    }

    public void setFireControl(MotionMagicVelocityTorqueCurrentFOC control) {
        shooterMotor1.setControl(control);
        shooterMotor2.setControl(control);
    }

    public void setAngleControl(MotionMagicVelocityTorqueCurrentFOC control) {
        angleMotor1.setControl(control);
        angleMotor2.setControl(control);
    }

    public double getVelocity() {
        return (shooterMotor1.getVelocity().getValueAsDouble() + shooterMotor2.getVelocity().getValueAsDouble()) / 2;
    }

    public void resetMotor() {
        shooterMotor1.setPosition(0);
        shooterMotor2.setPosition(0);
    }

    public void setFireNeutralMode(NeutralModeValue mode) {
        shooterMotor1.setNeutralMode(mode);
        shooterMotor2.setNeutralMode(mode);
    }

    public void setAngleNeutralMode(NeutralModeValue mode) {
        angleMotor1.setNeutralMode(mode);
        angleMotor2.setNeutralMode(mode);
    }

    public NeutralModeValue getFireNeutralMode() {
        return currentFireNeutralMode;
    }

    public NeutralModeValue getAngleNeutralMode() {
        return currentAngleNeutralMode;
    }

    @Override
    public void periodic() {
        this.shooterMotor1.set(velocity);
        this.shooterMotor2.set(velocity);
        this.angleMotor1.set(angle);
        this.angleMotor2.set(angle);

        if (getFireNeutralMode() != fireNeutralModeChooser.getSelected()) {
            setFireNeutralMode(fireNeutralModeChooser.getSelected());
            currentFireNeutralMode = fireNeutralModeChooser.getSelected();
        }

        if (getAngleNeutralMode() != angleNeutralModeChooser.getSelected()) {
            setAngleNeutralMode(angleNeutralModeChooser.getSelected());
            currentAngleNeutralMode = angleNeutralModeChooser.getSelected();
        }
    }

}
