package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** This command exists largely to allow the 
 *  use of intake.intake() as a default command */
public class IntakePiece extends Command {
    private Intake intake;
    private double speed;
    
    public IntakePiece(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }
   
    @Override
    public void initialize() {}

    @Override
    public void execute() {
        intake.intake(speed);
    }

    
    @Override 
    public boolean isFinished() {
		return false; 
    }

        
    @Override 
    public void end(boolean interrupted) {}
}
