package frc.robot.subsystems;


import com.ctre.phoenix6.controls.MotionMagicTorqueCurrentFOC;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.controller.*;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Shooter extends SubsystemBase {
    private static Shooter instance;
    private TalonFX shooterMotor1;
    private TalonFX shooterMotor2;
    private TalonFX angleMotor1;
    private TalonFX angleMotor2;
    private double velocity = 0.0;
    private double angleTarget = 0.0;
    private double angleCurrent = 0.0;
    private boolean isAtVelocity = false;
    public double getAngleCurrent() {
        return angleCurrent;
    }

    public void setAngleCurrent(double angleCurrent) {
        this.angleCurrent = angleCurrent;
    }

    private SendableChooser<NeutralModeValue> angleNeutralModeChooser = new SendableChooser<>();
    private SendableChooser<NeutralModeValue> fireNeutralModeChooser = new SendableChooser<>();
    private MotionMagicVelocityTorqueCurrentFOC fireControl;
    private MotionMagicVelocityTorqueCurrentFOC lastFireControl;
    private MotionMagicTorqueCurrentFOC angleControl;
    private NeutralModeValue currentFireNeutralMode = Constants.Swerve.Shooter.fireNeutralMode;
    private NeutralModeValue currentAngleNeutralMode = Constants.Swerve.Shooter.angleNeutralMode;
    private BinarySensor angleSensor;
    private SimpleMotorFeedforward shooterFeedforward = new SimpleMotorFeedforward(Constants.Swerve.Shooter.shootkS, Constants.Swerve.Shooter.shootkV, Constants.Swerve.Shooter.shootkA);
    private ArmFeedforward angleFeedForward = new ArmFeedforward(Constants.Swerve.Shooter.anglekS, Constants.Swerve.Shooter.anglekG, Constants.Swerve.Shooter.anglekV);

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
        this.angleControl = new MotionMagicTorqueCurrentFOC(angleTarget, 0.0, 0, false, false, false);
        this.shooterMotor1.setControl(fireControl);
        this.shooterMotor2.setControl(fireControl);
        this.angleMotor1.setControl(angleControl);
        this.angleMotor2.setControl(angleControl);
        this.angleSensor = new BinarySensor(Constants.Swerve.Shooter.angleSensorPort);

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

    public void setAngleTarget(double angle) {
        this.angleTarget = angle;
    }

    public void stop() {
        velocity = 0.0;
    }

    public void spin() {

    }

    public void setFireControl(MotionMagicVelocityTorqueCurrentFOC control) {
        shooterMotor1.setControl(control);
        shooterMotor2.setControl(control);
    }

    public MotionMagicVelocityTorqueCurrentFOC getFireControl() {
        return fireControl;
    }

    public void setAngleControl(MotionMagicTorqueCurrentFOC control) {
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
        if(Math.abs(velocity - (shooterMotor1.getVelocity().getValueAsDouble() + shooterMotor2.getVelocity().getValueAsDouble())/2) < 0.5) {
            isAtVelocity = true;
        }
        else {
            isAtVelocity = false;
        }
        if(fireControl != lastFireControl) {
            fireControl = lastFireControl;
        }

        if(angleSensor.get()) {
            angleCurrent = 0.0;
        }
        else {
            angleCurrent = (angleMotor1.getPosition().getValueAsDouble() + angleMotor2.getPosition().getValueAsDouble()) / 2;
        }

        setFireControl(fireControl.withFeedForward(shooterFeedforward.calculate(velocity)));
        setAngleControl(angleControl.withFeedForward(angleFeedForward.calculate(angleCurrent, velocity)));

        this.shooterMotor1.set(velocity);
        this.shooterMotor2.set(velocity);
        this.angleMotor1.set(angleTarget);
        this.angleMotor2.set(angleTarget);

        if (getFireNeutralMode() != fireNeutralModeChooser.getSelected()) {
            setFireNeutralMode(fireNeutralModeChooser.getSelected());
            currentFireNeutralMode = fireNeutralModeChooser.getSelected();
        }

        if (getAngleNeutralMode() != angleNeutralModeChooser.getSelected()) {
            setAngleNeutralMode(angleNeutralModeChooser.getSelected());
            currentAngleNeutralMode = angleNeutralModeChooser.getSelected();
        }

        lastFireControl = fireControl;
    }

}
