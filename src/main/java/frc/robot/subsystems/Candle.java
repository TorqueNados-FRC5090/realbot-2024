package frc.robot.subsystems;
import com.ctre.phoenix.led.Animation;
import com.ctre.phoenix.led.CANdle;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Candle extends SubsystemBase {

    public CANdle candle;

    /** */
    public Candle(int ID){
        candle = new CANdle(ID);
    }

    public void setRed(){
        candle.setLEDs(255, 0, 0);
    }

    public void setYellow(){
        candle.setLEDs(255,100,0);
    }

    public void setBlue(){
        candle.setLEDs(0, 10, 181);
    }

    public void setGreen(){
        candle.setLEDs(25, 255, 0);
    }

    public void setPurple(){
        candle.setLEDs(162, 18, 184);
    }
}
