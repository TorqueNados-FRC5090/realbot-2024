package frc.robot.subsystems;
import com.ctre.phoenix.led.CANdle;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.LEDConstants.LEDColor;

public class Candle extends SubsystemBase {

    public CANdle candle;

    /** */
    public Candle(int ID){
        candle = new CANdle(ID);
    }

    public void setAll(LEDColor color) {
        candle.setLEDs(color.getRed(), color.getGreen(), color.getBlue());
    }
}
