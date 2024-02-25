package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Shooter;

public class SetShooterState extends Command{
    /** This command moves the shooter to a given position and ends */
    private Shooter shooter;
    private ShooterPosition position;
    private double speed;

    /** Constructs a SetshooterPostion command
     *  @param shooter The shooter subsystem to control
     *  @param position The position the shooter should go to 
     *  @param speed The speed the shooter should rev up to in RPM */
    public SetShooterState(Shooter shooter, ShooterPosition position, double speed){
        this.shooter = shooter;
        this.position = position;
        this.speed = speed;
        
        addRequirements(shooter);
    }

    @Override // Command the shooter pid controller
    public void initialize() {
        shooter.goToPosition(position);
        shooter.setSpeed(speed);
    }

    @Override
    public void execute() {}

    @Override // Command ends when the shooter is in position
    public boolean isFinished() {
        return shooter.atTargetPosition() && shooter.atTargetSpeed();
    }
   
    public void end(boolean interrupted) {}
}
