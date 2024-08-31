package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;

public abstract class AdjustLetterCommand<T extends Subsystem> extends Command {
    private final TunePIDCommand<T> controller;
    protected final PIDController pidController;

    protected static final double K_INCREMENT = 0.01; // Increment for tuning P
    private double lastError = 0.0;
    private double accumulatedError = 0.0;
    private double lastDerivative = 0.0;

    public AdjustLetterCommand(TunePIDCommand<T> controller, PIDController pidController) {
        this.controller = controller;
        this.pidController = pidController;
        if (controller.HasRequirement()) {
            addRequirements(controller.GetRequirement());
        }
    }

    @Override
    public void initialize() {
        lastError = pidController.getSetpoint() - controller.GetMeasurement();
        accumulatedError = 0.0;
        lastDerivative = 0.0;
    }

    public abstract void setLetter(double error, double accumulatedError, double lastError, double derivative,
            double lastDerivative);

    @Override
    public void execute() {
        double error = pidController.getSetpoint() - controller.GetMeasurement();
        double derivative = error - lastDerivative;
        accumulatedError += error;

        setLetter(error, accumulatedError, lastError, derivative, lastDerivative);

        lastDerivative = derivative;
        lastError = error;
        double PIDOutput = pidController.calculate(controller.GetMeasurement());
        controller.SetVelocity(PIDOutput);
    }

    @Override
    public void end(boolean interrupted) {
        controller.Stop();
    }
}
