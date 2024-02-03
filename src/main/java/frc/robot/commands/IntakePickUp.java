package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.IntakeConstants.IntakePosition;
import frc.robot.subsystems.Intake;

public class IntakePickUp extends Command{
    private Intake intake;
    
    public IntakePickUp(Intake intake){
        this.intake = intake;
        addRequirements(intake);

        this.beforeStarting(new SetIntakePosition(intake, IntakePosition.PICKUP));
        this.andThen(new SetIntakePosition(intake, IntakePosition.SHOOT));
    }

    /**
     * Intake starts
     */
    @Override
    public void execute(){
        intake.intake(.25);
    }
    /**
     * stops intake when game piece is in
     */
    @Override 
    public boolean isFinished(){
        return intake.holdingPiece();
    }
    /** 
     * Intake goes to shoot position
     */
    @Override 
    public void end(boolean interrupted){
        intake.stopIntake();
    }

}
