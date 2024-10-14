package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.util.*;

public class TunePIDCommand<T extends Subsystem> extends SequentialCommandGroup {
    private final PIDController pidController;

    private static final double kP_INITIAL = 0.1; // Initial guess for kP
    private static final double kI_INITIAL = 0.0; // Initial guess for kI
    private static final double kD_INITIAL = 0.0; // Initial guess for kD

    private double currentKP = kP_INITIAL;
    private double currentKI = kI_INITIAL;
    private double currentKD = kD_INITIAL;

    private static final double TARGET_SETPOINT = 1.0; // Target setpoint for the subsystem

    protected T requirement = null;
    protected Get<T,Double> getMeasurementFromSubsystem;
    protected DiFunc<T,Double> setVelocity;

    public TunePIDCommand(T requirement, Get<T,Double> getMeasurementFromSubsystem, DiFunc<T,Double> setVelocity) {
        this.requirement = requirement;
        this.getMeasurementFromSubsystem = getMeasurementFromSubsystem;
        this.setVelocity = setVelocity;
        this.pidController = new PIDController(currentKP, currentKI, currentKD);
        pidController.setSetpoint(TARGET_SETPOINT);

        addRequirements(requirement);

        // Add nested commands with timeouts to gauge responses
        addCommands(
            new AdjustPCommand<T>(this, pidController).withTimeout(20.0), // Adjust P, wait for 2 seconds
            new AdjustICommand<T>(this, pidController).withTimeout(20.0), // Adjust I, wait for 2 seconds
            new AdjustDCommand<T>(this, pidController).withTimeout(20.0)  // Adjust D, wait for 2 seconds
        );
    }

    public boolean HasRequirement()
    {
        return requirement != null;
    }

    public Subsystem GetRequirement()
    {
        return requirement;
    }

    public double GetMeasurement()
    {
        return getMeasurementFromSubsystem.get(requirement);
    }

    public void SetVelocity(double value)
    {
        Double input = value;
        setVelocity.run(requirement,input);
    }

    public void Stop()
    {
        SetVelocity(0);
    }
}