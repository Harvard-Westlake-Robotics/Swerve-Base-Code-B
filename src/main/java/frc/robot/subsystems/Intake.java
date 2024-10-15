package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private TalonFX intakeMotor;
    private BinarySensor intakeSensor;
    private double velocity = 0.0;
    private final SimpleMotorFeedforward feedForward;
    private int intakeSpeed = 10;
    private int outtakeSpeed = -10;

    public int getIntakeSpeed() {
        return intakeSpeed;
    }

    public void setIntakeSpeed(int intakeSpeed) {
        this.intakeSpeed = intakeSpeed;
    }

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        this.intakeMotor = new TalonFX(Constants.Swerve.Intake.intakeMotorID);
        this.intakeMotor.getConfigurator().apply(new Slot0Configs().withKP(Constants.Swerve.Intake.intakeKP)
                .withKI(Constants.Swerve.Intake.intakeKI).withKD(Constants.Swerve.Intake.intakeKD));
        this.intakeMotor.setInverted(Constants.Swerve.Intake.intakeMotorInverted);
        this.intakeSensor = new BinarySensor(Constants.Swerve.Intake.intakeSensorPort);
        this.feedForward = new SimpleMotorFeedforward(Constants.Swerve.Intake.kS, Constants.Swerve.Intake.kV,
                Constants.Swerve.Intake.kA);
        this.intakeMotor.setNeutralMode(Constants.Swerve.Intake.intakeNeutralMode);
        this.intakeMotor.setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true,
                feedForward.calculate(velocity), 0, false, true, false));
    }

    public void setVelocity(double speed) {
        velocity = speed;
    }

    public void intake() {
        velocity = intakeSpeed;
    }

    public void outtake() {
        velocity = outtakeSpeed;
    }

    public void stop() {
        velocity = 0.0;
    }

    public BinarySensor getIntakeSensor() {
        return intakeSensor;
    }

    @Override
    public void periodic() {
        this.intakeMotor.set(velocity);
    }
}
