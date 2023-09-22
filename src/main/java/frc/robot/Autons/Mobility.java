package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.RobotContainer;

public class Mobility extends CommandBase {
    private final DriveBase m_driveBase;
    private final Manipulator m_manipulator;

    int counter = 0;

    public Mobility(DriveBase drive, Manipulator manip) {
        m_driveBase = drive;
        m_manipulator = manip;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0.0, 0.5, 0.0));
        while (counter <= RobotContainer.findClockTime(1.0)) {
            counter++;
        }
        counter = 0;
        m_driveBase.setDriveSpeed(RobotContainer.getSaturatedSpeeds(0.0, 0.0, 0.0));
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveBase.resetDrive();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
