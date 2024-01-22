package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

public class Shooter {
    private CANSparkMax rightMotor;
    private CANSparkMax leftMotor;

    public Shooter(int rightMotorID, int leftMotorID) {
        rightMotor = new CANSparkMax(rightMotorID, MotorType.kBrushless);
        rightMotor.restoreFactoryDefaults();
        
        leftMotor = new CANSparkMax(leftMotorID, MotorType.kBrushless);
        leftMotor.restoreFactoryDefaults();
        leftMotor.setInverted(true);
    }
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
