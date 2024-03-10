package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.wrappers.GenericPID;

public class AmpDeflector extends SubsystemBase{

    private CANSparkMax rotationMotor;
    private GenericPID ampPID;

    public AmpDeflector(int rotationID){
        rotationMotor = new CANSparkMax(rotationID, MotorType.kBrushless);
        rotationMotor.restoreFactoryDefaults();
        rotationMotor.setInverted(true);

        ampPID = new GenericPID(rotationMotor, ControlType.kPosition, 0.035);
        ampPID.activate(0);
    }

    public double getPostion() { return ampPID.getMeasurement();}

    public void goToPosition(double pos){
        ampPID.activate(pos);
    }

    public void stop(){
        ampPID.pause();
    }

    public Command deflectorOut() {
        return this.runEnd(() -> goToPosition(11.25), () -> goToPosition(0));
    }

    public Command deflectorOutFor(double seconds) {
        return this.runEnd(() -> goToPosition(11.25), () -> goToPosition(0)).withTimeout(seconds);
    }

    @Override // Called every 20ms
    public void periodic(){
        SmartDashboard.putNumber("Deflector Position", getPostion());
    }
}
