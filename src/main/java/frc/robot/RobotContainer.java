package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.commands.PathPlannerAuto;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

import frc.robot.commands.*;
import frc.robot.subsystems.*;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    /* Controllers */
    private final PS4Controller driver = new PS4Controller(0);
    /* Driver8Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, XboxController.Button.kY.value);
    private final JoystickButton robotCentric = new JoystickButton(driver,
            XboxController.Button.kLeftBumper.value);
    private final JoystickButton intakeButton = new JoystickButton(driver, PS4Controller.Button.kCircle.value);
    private static final boolean isDrifting = false;
    private final JoystickButton spinShooter = new JoystickButton(driver, PS4Controller.Button.kL2.value);
    private final JoystickButton angleButton = new JoystickButton(driver, PS4Controller.Button.kL1.value);

    public static boolean isDrifting() {
        return isDrifting;
    }

    /* Subsystems */
    private final Swerve s_Swerve = Swerve.getInstance();
    private final Intake intake = Intake.getInstance();
    private final Carriage carriage = Carriage.getInstance();
    private final Shooter shooter = Shooter.getInstance();

    /**
     * The container for the robot. Contains subsystems, OI devices, and commands.
     */
    public RobotContainer() {
        // s_Swerve.setDefaultCommand(
        // new TeleopSwerve(
        // s_Swerve,
        // () -> -driver.getRawAxis(translationAxis),
        // () -> -driver.getRawAxis(strafeAxis),
        // () -> -driver.getRawAxis(rotationAxis),
        // () -> robotCentric.getAsBoolean()));
        s_Swerve.setDefaultCommand(new TeleopSwerve(
                s_Swerve,
                () -> -driver.getLeftY(),
                () -> -driver.getLeftX(),
                () -> -driver.getRightX(), robotCentric));

        AutoBuilder.configureHolonomic(s_Swerve::getPose,
                s_Swerve::setPose,
                s_Swerve::getRobotVelocity,
                s_Swerve::fromChassisSpeeds, Constants.AutoConstants.getPathFollowerConfig(), RobotContainer::getIsRed,
                s_Swerve);

        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be
     * created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
     * it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroHeading()));
        // isDrifting.onTrue(new InstantCommand((() -> s_Swerve.isDrifting =
        // !s_Swerve.isDrifting)));
        intakeButton.onTrue(new Command() {
            @Override
            public void initialize() {
                intake.intake();
                carriage.intake();
            }

            @Override
            public void execute() {
                intake.setVelocity(carriage.getVelocity());
                carriage.intake();
                if (carriage.getNoteSensor().justEnabled() || carriage.isHasNote()) {
                    this.cancel();
                }
            }

            @Override
            public void end(boolean interrupted) {
                intake.stop();
                carriage.stop();
            }
        }.withTimeout(5));
        spinShooter.onTrue(new InstantCommand(() -> shooter.toggleShooter()));
        angleButton.onTrue(new InstantCommand(() -> {
            shooter.setAngleTarget(0.5);
        }));
    }

    public static boolean getIsRed() {
        var alliance = DriverStation.getAlliance();
        if (alliance.isPresent()) {
            return alliance.get() == DriverStation.Alliance.Red;
        }
        return false;
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        // An ExampleCommand will run in autonomous
        return new PathPlannerAuto("Splean Time");
    }
}
