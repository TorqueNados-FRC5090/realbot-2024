package frc.robot;


// Camera imports
import edu.wpi.first.cameraserver.CameraServer;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import static frc.robot.Constants.ShooterIDs.*;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import static frc.robot.Constants.IntakeIDs.*;

// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;
    private XboxController xbox;
    private Shooter shooter;
    private Intake intake;
    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Start the camera feed
        CameraServer.startAutomaticCapture();

        // Construct objects
        robotContainer = new RobotContainer();
        shooter = new Shooter(SHOOTER_RIGHT_ID, SHOOTER_LEFT_ID);
        xbox = new XboxController(0);
        intake = new Intake(INTAKE_DRIVER_ID, INTAKE_ROTATOR_ID, LASER_ID);
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
        if(xbox.getBButton())
            shooter.shoot(.5);

        else if(xbox.getAButton())
            shooter.stop();
            

        if(xbox.getYButton())
            shooter.shootOffset(.25, .5);
        
        else if(xbox.getLeftTriggerAxis() > 0.05)
            intake.eject(xbox.getLeftTriggerAxis());
        
        else if(xbox.getRightTriggerAxis()>.05)
            intake.intake(xbox.getRightTriggerAxis());
        
        else {
            intake.stop();
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