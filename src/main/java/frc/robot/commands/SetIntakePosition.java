package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class SetIntakePosition extends Command{
    private Intake intake;
    private IntakePosition targetPosition;

    public  SetIntakePosition(Intake intake, IntakePosition targetPosition){
        this.intake = intake;

        this.targetPosition = targetPosition;
        
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.goTo(targetPosition);
    }

    @Override
    public void execute(){
    }

    @Override 
    public boolean isFinished(){
        return intake.intakeAtSetPoint();
    }
   
    public void end(boolean interrupted){

    }
}
