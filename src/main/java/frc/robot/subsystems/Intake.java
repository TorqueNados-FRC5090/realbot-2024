package frc.robot.subsystems;

import static frc.robot.Constants.IntakeConstants.*;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants.IntakePosition;
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
        rotationMotor.setIdleMode(IdleMode.kBrake);
        rotationPID = new GenericPID(rotationMotor, ControlType.kPosition, 0.06, ROTATION_MOTOR_RATIO);

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
        SmartDashboard.putNumber("Intake Position Degrees", rotationPID.getPositionNoRatio());
    }
}
