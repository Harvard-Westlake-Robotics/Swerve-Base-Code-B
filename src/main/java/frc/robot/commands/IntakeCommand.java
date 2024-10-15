package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Carriage;
import frc.robot.subsystems.Intake;

public class IntakeCommand extends Command {
    // todo Implement PathPlanner Intake Command
    public IntakeCommand() {
    }

    @Override
    public void initialize() {
        Intake.getInstance().intake();
        Carriage.getInstance().intake();
    }

    @Override
    public void execute() {
        if(Carriage.getInstance().isHasNote()){
            this.cancel();
        }
        else{
            
        }
    }


    @Override
    public void end(boolean interrupted) {
        Intake.getInstance().stop();
        Carriage.getInstance().stop();
    }
}
