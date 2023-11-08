package frc.robot.commands;

import java.nio.file.Path;
import java.util.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.CommandBase;

import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;

public class AutonLoader {
    private static HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private final DriveBase m_driveBase;
    private final Manipulator m_manipulator;
    private final AutonCommands m_autoncommands;
    private final Elevator m_elevator;
    private static SendableChooser<Command> chooser;

    //PathPlanner auton groups
    private static List<PathPlannerTrajectory> trajectory = PathPlanner.loadPathGroup("trajectory", Constants.Swerve.AUTON_CONSTRAINTS);
    private static List<PathPlannerTrajectory> sf8 = PathPlanner.loadPathGroup("sf8", Constants.Swerve.AUTON_CONSTRAINTS);

    public AutonLoader(DriveBase driveBase, Manipulator manipulator, Elevator elevator) {

        m_driveBase = driveBase;
        m_manipulator = manipulator;
        m_elevator = elevator;
        m_autoncommands = new AutonCommands(driveBase, manipulator, elevator);
        chooser = new SendableChooser<>();

        eventMap = new HashMap<>();
        eventMap.put("event1", m_autoncommands.ELEVATIONUP);
        eventMap.put("event2", m_autoncommands.OUTTAKE);
        eventMap.put("event3", m_autoncommands.ELEVATIONDOWN);

        autoBuilder = new SwerveAutoBuilder(
                m_driveBase::getCurrentPose,
                m_driveBase::resetOdometry,
                Constants.DriveTrainConstants.TRANSLATION_PID,
                Constants.DriveTrainConstants.ROTATION_PID,
                m_driveBase::setAutoSpeed,
                eventMap,
                m_driveBase);
        
        // for (String path : Constants.Auton.paths) {
            // chooser.addOption(path, getAutonFromPath(path));
        // }

        // chooser.addOption("Single Score Mobility", m_autoncommands.MOBILITY);
        // chooser.addOption("Double Score", m_autoncommands.SCORING);

        chooser.addOption("trajectory", autoBuilder.fullAuto(trajectory));
        chooser.addOption("sf8", autoBuilder.fullAuto(sf8));

        SmartDashboard.putData(chooser);
    }

    // private Command getAutonFromPath(String path) {
    //     return new PrintCommand(path);
    // }

    public Command getAuton() {
        return chooser.getSelected();
        // return autoBuilder.fullAuto(pathGroup);
    }    
}