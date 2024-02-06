package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

public class AlwaysIntake extends Command {
    private Intake intake;
    
    public AlwaysIntake(Intake intake){
        this.intake = intake;
        addRequirements(intake);
    }
   
    @Override
    public void initialize(){}

    @Override
    public void execute(){
        intake.intake(.25);
    }

    @Override 
    public boolean isFinished(){
		return false; }

        @Override 
    public void end(boolean interrupted){}
}
