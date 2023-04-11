package frc.robot.commands;

import java.util.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutonLoader {
    private static HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private static DriveBase m_DriveBase;
    private static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("trajectory", Constants.Swerve.AUTON_CONSTRAINTS);

    public AutonLoader(DriveBase driveBase) {
        eventMap = new HashMap<>();
        eventMap.put("event1", new PrintCommand("event 1 passed"));    
        eventMap.put("event1", new PrintCommand("event 2 passed"));    
        m_DriveBase = driveBase;

        autoBuilder = new SwerveAutoBuilder(
                driveBase::getCurrentPose,
                driveBase::resetOdometry,
                Constants.DriveTrainConstants.TRANSLATION_PID,
                Constants.DriveTrainConstants.ROTATION_PID,
                (chassisSpeeds) -> m_DriveBase.setDriveSpeed(chassisSpeeds),
                eventMap,
                m_DriveBase);
        
    }

    public Command getAuton() {
        return this.autoBuilder.fullAuto(pathGroup); 
        
        // TODO: add sequential commands for resetting gyro
    }

    
}