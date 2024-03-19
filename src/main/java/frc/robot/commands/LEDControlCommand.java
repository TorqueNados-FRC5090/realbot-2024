package frc.robot.commands;

import static frc.robot.Constants.LEDConstants.*;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Candle;

public class LEDControlCommand extends Command {
    private Intake intake;
    private Shooter shooter;
    private Candle candle;
    public LEDControlCommand(Intake intake, Shooter shooter, Candle candle) {
       this.intake = intake;
       this.shooter = shooter;
       this.candle = candle;
       addRequirements(candle);
    }

    @Override
    public void initialize() {
    // This command should turn on the LED at the start of the game
        
    }
    
    @Override
    public void execute() {
     // This command should change colors for if there is an object
        if (intake.holdingPiece() && shooter.readyToShoot() ) {
            candle.setAll(LEDColor.GREEN); }
        else if (intake.holdingPiece()) {
            candle.setAll(LEDColor.ORANGE); }
        else candle.setAll(LEDColor.BLUE);
    }
    @Override
    public boolean isFinished(){
        // This command should turn off the LED at the end of the game
        return false;
    }
}