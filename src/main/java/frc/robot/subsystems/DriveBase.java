package frc.robot.subsystems;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;


import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class DriveBase extends SubsystemBase {

    private static Module.ModuleState targetModuleStates[];
    private final Kinematics m_kinematics;

    public static Module[] moduleGroup;

    public static double[] odomDeltas = {0, 0, 0, 0};
    public static double[] odomPrevDeltas = {0, 0, 0, 0};
    public static double[] odomAngles = {0, 0, 0, 0};

    double mod1Prev = 0;
    double mod1Curr = 0;
    int counter = 0;

    public DriveBase(Kinematics kinematics) {
        m_kinematics = kinematics;

        moduleGroup = new Module[4];
        for (int i = 0; i < 4; i++)
            moduleGroup[i] = new Module(i, Constants.DriveTrainConstants.invertedMotors[i]);

        targetModuleStates = new Module.ModuleState[4];

        for (int i = 0; i < 4; i++)
            targetModuleStates[i] = new Module.ModuleState(0, Constants.MotorConstants.motorDegrees[i] * (Math.PI/180));
    } 

    public Pose2d getCurrentPose() {
        return new Pose2d();
        //TODO: FIX THIS
    }

    public void resetOdometry(Pose2d pose) {
        //TODO: FIX THIS
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
        // RobotContainer.m_photonsubsystem.updatePose();
        for (int i = 0; i < 4; i++) {
            moduleGroup[i].setSpeedAndAngle(targetModuleStates[i]);
            odomDeltas[i] = ((moduleGroup[i].driveMotor.getEncoder().getPosition()*4*Math.PI)-odomPrevDeltas[i])/0.02;
            odomPrevDeltas[i] = odomDeltas[i];
            odomAngles[i] = moduleGroup[i].getAngle();
        }

        SmartDashboard.putNumber("Mod1_dy/dx", odomDeltas[0]);
        SmartDashboard.putNumber("Mod2_dy/dx", odomDeltas[1]);
        SmartDashboard.putNumber("Mod3_dy/dx", odomDeltas[2]);
        SmartDashboard.putNumber("Mod4_dy/dx", odomDeltas[3]);


        // mod1Curr = moduleGroup[0].getAngle();
        // if ((counter++ % 50) == 0) {
        //     System.out.println(odomDeltas[0]);
        // }
        
        // mod1Prev = mod1Curr;
        
    }
}