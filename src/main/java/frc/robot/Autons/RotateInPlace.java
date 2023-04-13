// package frc.robot.Autons;

// import edu.wpi.first.wpilibj2.command.CommandBase;

// import frc.robot.ModuleUtilities.ModuleGroup;
// import frc.robot.subsystems.DriveBase;
// import frc.robot.subsystems.Systems;

// import edu.wpi.first.math.kinematics.ChassisSpeeds;

// public class RotateInPlace extends CommandBase {
//     private final ModuleGroup m_moduleGroup;
//     private final DriveBase m_driveBase;

//     public RotateInPlace() {
//         m_moduleGroup = Systems.moduleGroup;
//         m_driveBase = Systems.driveBase;
//     }

//     // Called when the command is initially scheduled.
//     @Override
//     public void initialize() {
//         m_moduleGroup.resetDrive();
//     }

//     // Called every time the scheduler runs while the command is scheduled.
//     @Override
//     public void execute() {
//         m_driveBase.setDriveSpeed(new ChassisSpeeds(0.0, 0.0, 1));
//     }

//     // Called once the command ends or is interrupted.
//     @Override
//     public void end(boolean interrupted) {
//         m_moduleGroup.resetDrive();
//     }

//     // Returns true when the command should end.
//     @Override
//     public boolean isFinished() {
//         return false;
//     }
// }
