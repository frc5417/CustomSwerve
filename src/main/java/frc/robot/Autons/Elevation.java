package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.RobotContainer;

public class Elevation extends CommandBase {
    private final DriveBase m_driveBase;
    private final Manipulator m_manipulator;
    private final Elevator m_elevator;

    int counter = 0;

    public Elevation(DriveBase drive, Manipulator manip, Elevator elevator) {
        m_driveBase = drive;
        m_manipulator = manip;
        m_elevator = elevator;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_elevator.UpAndAway(-0.375);
        m_manipulator.setIntake(-0.1);
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {}

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}