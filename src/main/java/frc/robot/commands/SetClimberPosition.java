package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ClimberConstants.ClimberPosition;
import frc.robot.subsystems.Climber;

/** This command moves the climber to a given position and ends */
public class SetClimberPosition extends Command {
    private Climber climber;
    private ClimberPosition pos;

    /** Constructs a SetClimberPostion command
     *  @param climber The climber subsystem to control
     *  @param targetPosition The height in inches the climber should extend to */
    public SetClimberPosition(Climber climber, ClimberPosition targetPosition){
        this.climber = climber;
        this.pos = targetPosition;
        
        addRequirements(climber);
    }

    @Override // Command the climber pid controller
    public void initialize() {
        climber.goToPosition(pos);
    }

    @Override
    public void execute() {}

    @Override // Command ends when the climber is in position
    public boolean isFinished() {
        return climber.atSetpoint();
    }
   
    public void end(boolean interrupted) {}
}

