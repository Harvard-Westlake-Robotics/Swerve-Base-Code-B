package frc.robot.commands;

import java.util.ArrayList;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Shooter;

public class ShooterPresetCommand extends Command {
    ArrayList<Double> presets = new ArrayList<>();
    public ShooterPresetCommand() {
        presets.add(15.0);
        presets.add(30.0);
        presets.add(45.0);
        presets.add(60.0);

    }

    @Override
    public void initialize() {
        if(Shooter.getInstance().getAngleTarget() == 60){
            Shooter.getInstance().setAngleTarget(15.0);
        }
        else if(presets.contains(Shooter.getInstance().getAngleTarget())){
            Shooter.getInstance().setAngleTarget(presets.get(presets.indexOf(Shooter.getInstance().getAngleTarget()) + 1));
        }
        else{
            Shooter.getInstance().setAngleTarget(15.0);
        }
    }

    @Override
    public void execute() {
        
    }

    @Override
    public void end(boolean interrupted) {
        
    }


    
}
