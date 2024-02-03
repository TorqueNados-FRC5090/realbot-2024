package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class SetIntakePosition extends Command{
    private Intake intake;

    public  SetIntakePosition(Intake intake, IntakePosition targetPosition){
        this.intake = intake;
        addRequirements(intake);
    }

    @Override
    public void initialize(){
        intake.goTo(IntakePosition.CLIMB);
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
