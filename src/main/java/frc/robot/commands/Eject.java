package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class Eject extends Command{
    private Intake intake;

    public Eject(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
    @Override
    public void initialize() {
        intake.eject(1);
    }

    @Override
    public void execute(){
        
    }

    @Override 
    public boolean isFinished(){
		return false; }

    @Override 
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
