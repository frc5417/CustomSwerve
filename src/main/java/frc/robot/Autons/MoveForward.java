package frc.robot.Autons;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveBase;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.Constants;
import frc.robot.subsystems.RobotContainer;

public class MoveForward extends CommandBase {
    private final DriveBase m_driveBase;

    int counter = 0;

    public MoveForward(DriveBase drive) {
        m_driveBase = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        m_driveBase.setDriveSpeed(new ChassisSpeeds(0.0*Constants.Swerve.XPercentage, 0.5*Constants.Swerve.YPercentage, 0.0*Constants.Swerve.angularPercentage));
        while (counter <= RobotContainer.findClockTime(1.0)) {
            counter++;
        }
        m_driveBase.setDriveSpeed(new ChassisSpeeds(0, 0, 0));
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
