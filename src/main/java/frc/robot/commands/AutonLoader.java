package frc.robot.commands;

import java.util.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;

public class AutonLoader {
    private static HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private final DriveBase m_driveBase;
    private final AutonCommands m_autoncommands;
    private static SendableChooser<Command> chooser;
    private static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("trajectory", Constants.Swerve.AUTON_CONSTRAINTS);

    public AutonLoader(DriveBase driveBase) {

        m_driveBase = driveBase;
        m_autoncommands = new AutonCommands(driveBase);
        chooser = new SendableChooser<>();

        eventMap = new HashMap<>();
        eventMap.put("event1", new PrintCommand("event 1 passed"));    
        eventMap.put("event2", new PrintCommand("event 2 passed"));    

        autoBuilder = new SwerveAutoBuilder(
                m_driveBase::getCurrentPose,
                m_driveBase::resetOdometry,
                Constants.DriveTrainConstants.TRANSLATION_PID,
                Constants.DriveTrainConstants.ROTATION_PID,
                m_driveBase::setDriveSpeed,
                eventMap,
                m_driveBase);
        
        // for (String path : Constants.Auton.paths) {
            // chooser.addOption(path, getAutonFromPath(path));
        // }
        chooser.addOption("forward", m_autoncommands.MOVE_FORWARD);

        SmartDashboard.putData(chooser);
    }

    private Command getAutonFromPath(String path) {
        return new PrintCommand(path);
    }

    public Command getAuton() {
        return chooser.getSelected();
    }    
}