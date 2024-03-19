package frc.robot.subsystems;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDColor;
import frc.robot.Constants.LEDConstants.LEDStrip;

public class Candle extends SubsystemBase {

    public CANdle candle;

    /** */
    public Candle(int ID){
        candle = new CANdle(ID);
    }

    public void setAll(LEDColor color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }

    public void setStrip(LEDColor color, LEDStrip strip) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue(), 0, 
            strip.getStartingIndex(), strip.getStripLength()); 
    }
}
