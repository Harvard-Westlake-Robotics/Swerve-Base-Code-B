package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;

public class AdjustPCommand<T extends Subsystem> extends AdjustLetterCommand<T> {
    public AdjustPCommand(TunePIDCommand<T> controller, PIDController pidController) {
        super(controller, pidController);
    }

    public void setLetter(double error, double accumulatedError, double lastError, double derivative, double lastDerivative)
    {
        if (Math.abs(error) > Math.abs(lastError)) {
            // Increase P if the error is not decreasing
            pidController.setP(pidController.getP() + K_INCREMENT);
        } else {
            // If the error is decreasing, we might be close to an optimal P
            pidController.setP(Math.max(0, pidController.getP() - K_INCREMENT));
        }
    }
}