package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Blinkin;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;

public class LEDControlCommand extends Command {
    private Intake intake;
    private Blinkin blinkin;
    private Shooter shooter;
    public LEDControlCommand(Intake intake, Blinkin blinkin, Shooter shooter) {
       this.intake = intake;
       this.blinkin = blinkin;
       this.shooter = shooter;
       addRequirements(blinkin);
    }

    @Override
    public void initialize() {
    // This command should turn on the LED at the start of the game
        
    }
    
    @Override
    public void execute() {
     // This command should change colors for if there is an object
        if (intake.holdingPiece() && shooter.readyToShoot()) {
            blinkin.blueBounce();}
        else if (intake.holdingPiece()) {
            blinkin.blueSolid(); }
        else blinkin.goldSolid();

        
    }
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return false;
    }
}