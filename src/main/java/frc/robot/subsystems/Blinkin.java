package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;

public class Blinkin {

    public Spark LED;

    public Blinkin(){
        LED = new Spark(0);
        blueBounce();
    }
    /*Blue bounce back and forth */
    public void blueBounce(){
        LED.set(-0.01);
    }
    
    public void heartbeatBlue(){
        LED.set(0.03);
    }

    public void goldSolid(){
        LED.set(0.67);
    }

    public void blueSolid(){
        LED.set(0.85);
    }

}
