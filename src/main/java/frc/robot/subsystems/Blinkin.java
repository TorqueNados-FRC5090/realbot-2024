package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Blinkin extends SubsystemBase  {

    public Spark LED;

    /** Creates a Blinkin
     *  @param port 
     *  PWM port used by the blinkin 
     */
    public Blinkin(int port){
        LED = new Spark(port);
    }
    /** Sets LED to blue, light bouncing back and fourth */
    public void blueBounce(){
        LED.set(-0.01);
    }
    /** Sets LED to blue, flashing softly */
    public void heartbeatBlue(){
        LED.set(0.03);
    }
    /** Sets LED to gold, solid color */
    public void goldSolid(){
        LED.set(0.67);
    }
    /** Sets LED to blue, solid color */
    public void blueSolid(){
        LED.set(0.85);
    }
    /** Sets LED to green, solid color */
    public void greenSolid(){
        LED.set(0.77);
    }
    /** Sets LED to red, solid color */
    public void redSolid(){
        LED.set(0.61);
    }
    /** Sets LED to orange, solid color */
    public void orangeSolid(){
        LED.set(0.63);
    }
    /** Sets LED to white, solid color */
    public void whiteSolid(){
        LED.set(0.93);
    }

}
