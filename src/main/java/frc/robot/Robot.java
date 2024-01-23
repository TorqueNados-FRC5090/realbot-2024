package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

// Camera imports
import edu.wpi.first.cameraserver.CameraServer;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.subsystems.Shooter;
// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;
    private XboxController xbox;
    private Shooter shooter;
    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct objects
        robotContainer = new RobotContainer();
        shooter = new Shooter(6, 1);
        xbox = new XboxController(0);
    }

    // This function is called once at the start of auton
    @Override
    public void autonomousInit() {
        // Get the command to be used in auton
        autonCommand = robotContainer.getAutonomousCommand();
        // Schedule the command if there is one
        if (autonCommand != null)
            autonCommand.schedule();
    }

    // This function is called every 20ms during auton
    @Override
    public void autonomousPeriodic() { }
    
    // This function is called once at the start of teleop
    @Override
    public void teleopInit() {
        // This makes sure that the autonomous command stops when teleop starts
        if (autonCommand != null)
            autonCommand.cancel();
    }

    // This function is called every 20ms during teleop
    @Override
    public void teleopPeriodic() {
        /*
        if(xbox.getAButton()){
            rightMotor.set(.25);
            leftMotor.set(-.25);
        }
        else if(xbox.getBButton()){
            rightMotor.set(.5);
            leftMotor.set(-.5);
        }
        else if(xbox.getYButton()){
            rightMotor.set(.75);
            leftMotor.set(-.75);
        }
        else if(xbox.getXButton()){
            rightMotor.set(1);
            leftMotor.set(-1);
        } 
        else if(xbox.getLeftBumper()){
            rightMotor.set(.4);
            leftMotor.set(-.6);
        }
        else if(xbox.getRightBumper()){
            rightMotor.set(.6);
            leftMotor.set(-.4);
        }
        else if (xbox.getRightTriggerAxis() > 0.05) {
            rightMotor.set(xbox.getRightTriggerAxis());
            leftMotor.set(-xbox.getRightTriggerAxis());
        }
        else {
            rightMotor.set(0);
            leftMotor.set(0);
        }
        */

        if(xbox.getBButton()){
            shooter.shoot(.5);
        }
        else if(xbox.getAButton()){
            shooter.stop();
        
        }
        else if(xbox.getYButton()){
            shooter.shootOffset(.25, .5);
        }
    }

    // This function is called every 20ms while the robot is enabled
    @Override
    public void robotPeriodic() {    
        // Run any functions that always need to be running
        CommandScheduler.getInstance().run();
    }

    @Override
    public void testPeriodic() {
    }
}