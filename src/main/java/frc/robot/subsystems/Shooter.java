package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.wrappers.GenericPID;

import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;
    private GenericPID shooterPID;

    public Shooter(int rightMotorID, int leftMotorID) {
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(true);

        //shooterPID = new GenericPID(leftMotor, ControlType.kVelocity, getLeftMotorRPM());

        SmartDashboard.putNumber("Left Motor RPM", getLeftMotorRPM());
        SmartDashboard.putNumber("Right Motor RPM", getRightMotorRPM());
    }

    public double getLeftMotorRPM() {return leftMotor.getEncoder().getVelocity(); }
    public double getRightMotorRPM() {return rightMotor.getEncoder().getVelocity(); }

    /** shoots out a piece
     * @param speed speed of motors by %
     */
    public void shoot(double speed) {
        rightMotor.set(speed);
        leftMotor.set(speed);
    }
    public void stop() {
        rightMotor.set(0);
        leftMotor.set(0);
    }
    /** Shoots piece at an angle
     * @param rightSpeed speed of right motor by %
     * @param leftSpeed spee of left motor by %
     */
    public void shootOffset(double rightSpeed, double leftSpeed){
        rightMotor.set(rightSpeed);
        leftMotor.set(leftSpeed);
    }
}
