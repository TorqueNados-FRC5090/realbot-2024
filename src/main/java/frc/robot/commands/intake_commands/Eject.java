package frc.robot.commands.intake_commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/** This command exists largely to use intake.eject while requiring the intake */
public class Eject extends Command{
    private Intake intake;
    private double speed;

    /**  */
    public Eject(Intake intake, double speed) {
        this.intake = intake;
        this.speed = speed;
        addRequirements(intake);
    }

    @Override // Attempt to eject any held piece
    public void initialize() {
        intake.eject(speed);
    }

    @Override // Do nothing until command ends
    public void execute() {}

    @Override // Command never ends on its own
    public boolean isFinished() {
		return false;
    }

    @Override // Stop the ejecting when the command ends
    public void end(boolean interrupted) {
        intake.stopIntake();
    }
}
