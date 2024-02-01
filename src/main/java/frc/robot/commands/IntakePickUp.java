package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class IntakePickUp extends Command{
    private Intake intake;
    
    public IntakePickUp(int intakeID, int rotateID, int laserPort){
        intake = new Intake(intakeID, rotateID, laserPort);
    }

    @Override
    public void initialize(){
       intake.goTo(IntakePosition.PICKUP);
    }  

    @Override
    public void execute(){
        intake.intake(.25);
    }
    @Override 
    public boolean isFinished(){
        return intake.holdingPiece();
    }
    
    @Override 
    public void end(boolean interrupted){
        intake.goTo(IntakePosition.SHOOT);
    }

}
