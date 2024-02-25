package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class SetShooterPosition extends Command{
    /** This command moves the shooter to a given position and ends */
    private Shooter shooter;
    private ShooterPosition shooterPosition;

    /** Constructs a SetshooterPostion command
     *  @param shooter The shooter subsystem to control
     *  @param targetHeight The height in inches the shooter should extend to */
    public SetShooterPosition(Shooter shooter, ShooterPosition shooterPosition){
        this.shooter = shooter;
        this.shooterPosition = shooterPosition;
        
        addRequirements(shooter);
    }

    @Override // Command the shooter pid controller
    public void initialize() {
        shooter.goToPosition(shooterPosition);
    }

    @Override
    public void execute() {}

    @Override // Command ends when the shooter is in position
    public boolean isFinished() {
        return shooter.atTargetPosition();
    }
   
    public void end(boolean interrupted) {}
}
