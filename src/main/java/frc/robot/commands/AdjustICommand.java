package frc.robot.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Subsystem;


public class AdjustICommand<T extends Subsystem> extends AdjustLetterCommand<T> {
    public AdjustICommand(TunePIDCommand<T> controller, PIDController pidController) {
        super(controller, pidController);
    }

    public void setLetter(double error, double accumulatedError, double lastError, double derivative, double lastDerivative)
    {
        if (Math.abs(accumulatedError) > 0.05) {
            // Increase I if there's a steady error accumulation
            pidController.setI(pidController.getI() + K_INCREMENT);
        } else {
            // If no significant steady-state error, decrease I slightly
            pidController.setI(Math.max(0, pidController.getI() - K_INCREMENT));
        }
    }
}
