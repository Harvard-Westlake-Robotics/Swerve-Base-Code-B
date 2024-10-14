package frc.robot.subsystems;

import com.ctre.phoenix6.controls.MotionMagicVelocityTorqueCurrentFOC;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.util.BinarySensor;

public class Intake extends SubsystemBase {
    private static Intake instance;
    private TalonFX intakeMotor;
    private BinarySensor intakeSensor;
    private double velocity = 0.0;
    private final double feedForward = 0.0;

    public static Intake getInstance() {
        if (instance == null) {
            instance = new Intake();
        }
        return instance;
    }

    private Intake() {
        this.intakeMotor = new TalonFX(Constants.Swerve.Intake.intakeMotorID);
        this.intakeMotor.setInverted(Constants.Swerve.Intake.intakeMotorInverted);
        this.intakeSensor = new BinarySensor(Constants.Swerve.Intake.intakeSensorPort);
    }

    public void setIntakeSpeed(double speed) {
        velocity = speed;
    }

    @Override
    public void periodic() {
        this.intakeMotor
                .setControl(new MotionMagicVelocityTorqueCurrentFOC(velocity, 0.0, true, 0.0, 0, false, true, false));
    }
}
