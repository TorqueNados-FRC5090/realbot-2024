package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkFlex;
import com.revrobotics.CANSparkLowLevel;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.wrappers.LaserDetector;

public class Intake {
    private CANSparkMax intakeMotor;
    private CANSparkMax rotateMotor;
    private LaserDetector laser;

    public Intake(int intakeID, int rotateID, int laserPort){
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        
        rotateMotor = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotateMotor.restoreFactoryDefaults();

        laser = new LaserDetector(laserPort);
    }
    /** takes in a game piece
     * @param d speed of motor in %
     */
    public void intake(double d){
        if(d < 0){
            d = Math.abs(d);
        }

        if(laser.isOpen()) {
            intakeMotor.set(d);
        }
        else {
            stop();
        }
        
    }
    /** spits out a game piece
     * @param d speed of motor in %
     */
    public void eject(double d){
        if(d < 0){
            d = Math.abs(d);
        }
        intakeMotor.set(-d);
    }
    public void stop(){
        intakeMotor.set(0);
    }
    
    
}
