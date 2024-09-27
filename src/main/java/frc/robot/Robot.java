package frc.robot;

// Command imports
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.ShooterConstants.ShooterPosition;
// Misc imports
import edu.wpi.first.wpilibj.TimedRobot;


public class Robot extends TimedRobot {
    private RobotContainer robotContainer;

    // Commands
    private Command autonCommand;

    // This function is run when the robot is first started up and should be used
    // for any initialization code.
    @Override
    public void robotInit() {
        // Construct objects
        robotContainer = new RobotContainer();
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
        if (autonCommand != null) {
            autonCommand.cancel();
            robotContainer.shooter.goToPosition(ShooterPosition.POINT_BLANK);
            robotContainer.shooter.setSpeed(0);
        }
    }

    // This function is called every 20ms during teleop
    @Override
    public void teleopPeriodic() {}

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