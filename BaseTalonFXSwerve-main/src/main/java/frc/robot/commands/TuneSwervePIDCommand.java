package frc.robot.commands;

import frc.robot.subsystems.Swerve;

public class TuneSwervePIDCommand extends TunePIDCommand<Swerve>
{
    public TuneSwervePIDCommand(Swerve swerve)
    {
        super(swerve, (Swerve s) -> s.getMeasurement() , (Swerve s, Double value) -> { s.setVelocity(value); });
    }
}
