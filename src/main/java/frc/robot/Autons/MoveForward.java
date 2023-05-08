package frc.robot.autons;

import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.subsystems.DriveBase;

import edu.wpi.first.math.kinematics.ChassisSpeeds;

public class MoveForward extends CommandBase {
    private final DriveBase m_driveBase;
    private boolean doFinish = false;
    private int count = 0;

    public MoveForward(DriveBase drive) {
        m_driveBase = drive;
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        m_driveBase.setDriveSpeed(new ChassisSpeeds(0.5, 0.0, 0.0));
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        if (count++ >= 50) {
            m_driveBase.setDriveSpeed(new ChassisSpeeds(0.0, 0.0, 0.0));
            doFinish = true;
        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        m_driveBase.setDriveSpeed(new ChassisSpeeds(0.0, 0.0, 0.0));
        m_driveBase.resetDrive();
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return doFinish;
    }
}
