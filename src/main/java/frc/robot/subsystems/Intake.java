package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.IntakePosition;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import frc.robot.wrappers.GenericPID;
import frc.robot.wrappers.LaserDetector;

public class Intake {
    private CANSparkMax intakeMotor;
    private CANSparkMax rotationMotor;
    private LaserDetector laser; 
    private GenericPID rotationPID;

    public Intake(int intakeID, int rotateID, int laserPort){
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        
        rotationMotor = new CANSparkMax(rotateID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationPID = new GenericPID(rotationMotor, ControlType.kPosition, 0.2);

        laser = new LaserDetector(laserPort);
    }

    public void intake(double speed){  
        if(laser.isOpen()) {
            intakeMotor.set(-speed);
        }
        else {
            stop();
        }
    }

    public void eject(double speed){
        intakeMotor.set(speed);
    }

    public void stop(){
        intakeMotor.set(0);
    }

    public void goTo(IntakePosition pos){
        rotationPID.activate(pos.getAngle());
    }

    public void manualRotate(double speed){
        rotationMotor.set(speed);
    }

}
