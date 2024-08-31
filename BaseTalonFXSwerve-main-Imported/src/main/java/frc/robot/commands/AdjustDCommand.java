package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AdjustDCommand<T extends Subsystem> extends AdjustLetterCommand<T> {
    public AdjustDCommand(TunePIDCommand<T> controller, PIDController pidController) {
        super(controller, pidController);
    }

    public void setLetter(double error, double accumulatedError, double lastError, double derivative, double lastDerivative)
    {
        if (Math.abs(derivative) > 0.05) {
            // Increase D if the system is oscillating
            pidController.setD(pidController.getD() + K_INCREMENT);
        } else {
            // Decrease D if the system is too damped or sluggish
            pidController.setD(Math.max(0, pidController.getD() - K_INCREMENT));
        }
    }
}
