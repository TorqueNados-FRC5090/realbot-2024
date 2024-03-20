package frc.robot.commands;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.RobotContainer;
import frc.robot.subsystems.Candle;

public class LEDControlCommand extends Command {
    private final Intake intake;
    private final Shooter shooter;
    private final Candle candle;
    public LEDControlCommand(Candle candle, RobotContainer robot) {
        intake = robot.intake;
        shooter = robot.shooter;
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
        if (DriverStation.isDisabled())
            candle.setAll(LEDColor.RED);
        else if (DriverStation.isAutonomousEnabled())
            candle.setAll(LEDColor.PURPLE);
        else if (intake.holdingPiece() && shooter.readyToShoot() ) {
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