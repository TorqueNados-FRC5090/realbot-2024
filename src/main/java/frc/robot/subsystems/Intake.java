package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;
import frc.robot.wrappers.LaserDetector;

public class Intake extends SubsystemBase{
    private CANSparkMax intakeMotor;
    private CANSparkMax rotationMotor;
    private LaserDetector laser; 
    private GenericPID rotationPID;

    public Intake(int intakeID, int rotateID, int laserPort){
        intakeMotor = new CANSparkMax(intakeID, MotorType.kBrushless);
        intakeMotor.restoreFactoryDefaults();
        intakeMotor.setInverted(true);
        
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
            stopIntake();
        }
    }

    public void eject(double speed){
        intakeMotor.set(speed);
    }

    public void stopRotate(){
        rotationPID.pause();
        rotationMotor.set(0);
    }
        
    public void stopIntake(){
        intakeMotor.set(0);
    }

    public void goTo(IntakePosition pos) {
        rotationPID.activate(pos.getAngle());
    }

    public void manualRotate(double speed){
        rotationMotor.set(speed);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Rotation Position", rotationMotor.getEncoder().getPosition());
    }
}
