package frc.robot.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.drivetrain.SwerveDrivetrain;

public class LimeDrive extends Command {
    private final SwerveDrivetrain drivetrain;
    private final boolean originalOrientation;
    private final Limelight limelight;
    private final double goalDistance;

    private final PIDController driveControllerX;
    private final PIDController driveControllerY;
    private final PIDController headingController;

    /**  */
    public LimeDrive(SwerveDrivetrain drivetrain, Limelight limelight, double goalDistance) {
        this.drivetrain = drivetrain;
        originalOrientation = drivetrain.isFieldCentric();
        this.limelight = limelight;
        this.goalDistance = goalDistance;

        driveControllerX = new PIDController(.7, 0, 0);
        driveControllerX.setTolerance(.1); // Allow for 5 centimeters of positional error

        driveControllerY = new PIDController(.7, 0, 0);
        driveControllerY.setTolerance(.1); // Allow for 5 centimeters of positional error

        headingController = new PIDController(.02, 0, .0005);
        headingController.setTolerance(2); // Allow for 2 degrees of rotational error
        headingController.enableContinuousInput(-180, 180); // -180 and 180 are the same heading

        addRequirements(drivetrain);
    }

    // Called once when command is first scheduled.
    @Override
    public void initialize() {
        drivetrain.setFieldCentric(false);
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (!limelight.hasValidTarget()) {
            drivetrain.drive(0, 0, 0);
            return;
        }

        // Get the location of the target from limelight
        double[] targetpose = limelight.getTargetPose();
        double targetX = targetpose[0];
        double targetDist = targetpose[2];
        double targetAngle = targetpose[4];


        // Preprocess the driving instructions
        double xRawOutput = driveControllerX.calculate(targetX, 0);
        double xClamped = MathUtil.clamp(xRawOutput, -1, 1);
        double x = MathUtil.applyDeadband(xClamped, .01);
        
        double yRawOutput = -driveControllerY.calculate(targetDist, goalDistance);
        double yClamped = MathUtil.clamp(yRawOutput, -1, 1);    
        double y = MathUtil.applyDeadband(yClamped, .01);
        
        double rotRawOutput = -headingController.calculate(targetAngle, 0);
        double rotClamped = MathUtil.clamp(rotRawOutput, -1, 1);
        double rotation = MathUtil.applyDeadband(rotClamped, .01);
               
        // Send the instructions to the drivetrain
        drivetrain.sendDrive(x, y, rotation, true);

        SmartDashboard.putNumber("X Input", targetX);
        SmartDashboard.putNumber("Y Input", targetDist);
        SmartDashboard.putNumber("Rot Input", targetAngle);

        SmartDashboard.putNumber("X Output", x);
        SmartDashboard.putNumber("Y Output", y);
        SmartDashboard.putNumber("Rot Output", rotation);

        SmartDashboard.putBoolean("X at Target", driveControllerX.atSetpoint());
        SmartDashboard.putBoolean("Y at Target", driveControllerY.atSetpoint());
        SmartDashboard.putBoolean("Rot at Target", headingController.atSetpoint());
    }

    // Command ends when robot is at its destination
    @Override
    public boolean isFinished() {
        return driveControllerX.atSetpoint() &&
               driveControllerY.atSetpoint() &&
               headingController.atSetpoint();
    }

    // Runs when the command ends
    @Override
    public void end(boolean interrupted) {
        drivetrain.setFieldCentric(originalOrientation);
    }
}
