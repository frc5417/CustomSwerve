package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;
    private Pose2d currentLocation = new Pose2d();

    public static Module[] moduleGroup;
      

    public DriveBase(Kinematics kinematics) {
        m_kinematics = kinematics;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++)
            moduleGroup[i] = new Module(i, 
            Constants.DriveTrainConstants.invertedMotors[i],
            Constants.DriveTrainConstants.wheelLocations[i]
        );

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));
    } 

    public Pose2d getCurrentPose() {
        return currentLocation;
    }

    private void updatePose() { // updates current position of the robot
        ChassisSpeeds currentChassisSpeed = m_kinematics.toChassisSpeeds(moduleGroup);
        double xVel = currentChassisSpeed.vxMetersPerSecond;
        double yVel = currentChassisSpeed.vyMetersPerSecond;
        double omega = currentChassisSpeed.omegaRadiansPerSecond;

        // TODO: MAKE SURE TO CONFIRM THAT AUTON WORKS WITH DEGREES

        currentLocation = new Pose2d(
            new Translation2d(
                currentLocation.getX() + (xVel * 0.02), 
                currentLocation.getY() + (yVel * 0.02)
            ),
            new Rotation2d(
                currentLocation.getRotation().getDegrees() + (omega * (Math.PI / 180.0) * 0.02)
            )
        );
    }

    public void resetOdometry(Pose2d pose) {
        currentLocation = pose;
    }

    public void setHardStates(Module.ModuleState[] targetState) {
        targetModuleStates = targetState;
    }

    public void setDriveSpeed(ChassisSpeeds chassisSpeeds) {
        targetModuleStates = m_kinematics.getComputedModuleStates(chassisSpeeds);
    }

    public void resetDrive() {
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].resetDriveAngleEncoder();
        }
    }

    @Override
    public void periodic() {
        for (int i = 0; i < 4; i++)
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
        updatePose();
    }
}