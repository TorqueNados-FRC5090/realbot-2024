package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Shooter;

public class AimShooterAtSpeaker extends Command {
    private final Shooter shooter;
    private final Limelight limelight;
    private boolean ends;
    public AimShooterAtSpeaker(Limelight limelight, Shooter shooter, boolean ends) {
        this.limelight = limelight;
        this.shooter = shooter;
        this.ends = ends;

        addRequirements(shooter);
    }

    @Override
    public void initialize() { }
    
    @Override
    public void execute() {
        double targetSpeed;
        double targetAngle;

        if (limelight.hasValidTarget()) {
            targetAngle = -0.7634 * limelight.getTargetY() + 14.723;
            targetSpeed = 5000;
        }
        else {
            targetAngle = ShooterPosition.POINT_BLANK.getAngle();
            targetSpeed = 3500;
        }

        if (Math.abs(shooter.getPositionSetpoint() - targetAngle) > .2)
            shooter.goToAngle(targetAngle);
        if (Math.abs(shooter.getRPMSetpoint() - targetSpeed) > 100)
            shooter.setSpeed(targetSpeed);
    }

    @Override
    public void end(boolean interrupted) {}
    
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return ends && shooter.readyToShoot();
    }
}