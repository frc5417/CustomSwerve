package frc.robot.commands;

import java.util.*;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.Elevator;

public class AutonLoader {
    private static HashMap<String, Command> eventMap;
    private final DriveBase m_driveBase;
    private final Manipulator m_manipulator;
    private final AutonCommands m_autoncommands;
    private final Elevator m_elevator;
    private static SendableChooser<Command> chooser;
    private final ReplanningConfig replanningConfig = new ReplanningConfig();
    private final HolonomicPathFollowerConfig holonomic_config = new HolonomicPathFollowerConfig(Constants.Swerve.maxModuleSpeed, Constants.Swerve.driveBaseRadius, replanningConfig);

    //PathPlanner auton groups
    // private static List<PathPlannerPath> trajectory = PathPlannerAuto.getPathGroupFromAutoFile("trajectory");
    // private static List<PathPlannerPath> sf8 = PathPlannerAuto.getPathGroupFromAutoFile("sf8");
    // private static List<PathPlannerPath> bozo = PathPlannerAuto.getPathGroupFromAutoFile("newsf");
    // private static List<PathPlannerPath> straightline = PathPlannerAuto.getPathGroupFromAutoFile("straightline");

    public AutonLoader(DriveBase driveBase, Manipulator manipulator, Elevator elevator) {

        m_driveBase = driveBase;
        m_manipulator = manipulator;
        m_elevator = elevator;
        m_autoncommands = new AutonCommands(driveBase, manipulator, elevator);
        chooser = new SendableChooser<>();

        eventMap = new HashMap<>();
        eventMap.put("intakebruh", m_autoncommands.INTAKE);
        eventMap.put("event1", m_autoncommands.ELEVATIONUP);
        eventMap.put("event2", m_autoncommands.OUTTAKE);
        eventMap.put("event3", m_autoncommands.ELEVATIONDOWN);




        AutoBuilder.configureHolonomic(m_driveBase::getCurrentPose, m_driveBase::resetOdometry, m_driveBase::getRelativeChassisSpeeds, m_driveBase::setAutoSpeed, holonomic_config, m_driveBase::shouldFlipPath, m_driveBase);

        // for (String path : Constants.Auton.paths) {
            // chooser.addOption(path, getAutonFromPath(path));
        // }

        // chooser.addOption("Single Score Mobility", m_autoncommands.MOBILITY);
        // chooser.addOption("Double Score", m_autoncommands.SCORING);

        // chooser.addOption("trajectory", AutoBuilder.followPath((PathPlannerPath) trajectory));
        // chooser.addOption("sf8", AutoBuilder.followPath((PathPlannerPath) sf8));
        // chooser.addOption("straightline", AutoBuilder.followPath((PathPlannerPath) straightline));
        // chooser.addOption("newsf", AutoBuilder.followPathWithEvents((PathPlannerPath) newsf));

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