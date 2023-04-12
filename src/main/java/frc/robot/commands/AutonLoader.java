package frc.robot.commands;

import java.util.*;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.auto.SwerveAutoBuilder;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.Constants;
import frc.robot.subsystems.DriveBase;
import frc.robot.subsystems.Systems;

public class AutonLoader {
    private static HashMap<String, Command> eventMap;
    private SwerveAutoBuilder autoBuilder;
    private final DriveBase m_driveBase;
    private static SendableChooser<Command> chooser;
    private static List<PathPlannerTrajectory> pathGroup = PathPlanner.loadPathGroup("trajectory", Constants.Swerve.AUTON_CONSTRAINTS);

    public AutonLoader(Systems systems) {
        m_driveBase = systems.driveBase;
        
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
        
        for (String path : Constants.Auton.paths)
            chooser.addOption(path, getAutonFromPath(path));
    }

    private Command getAutonFromPath(String path) {
        switch (path) {
            case "rotateInPlace": 
                return new Command() {

                    @Override
                    public void schedule() {
                        m_driveBase.setDriveSpeed(new ChassisSpeeds(0.0, 0.0, 1));
                    }

                    @Override
                    public Set<Subsystem> getRequirements() {
                        Set<Subsystem> ret = new TreeSet<>();
                        ret.add(m_driveBase);
                        return ret;
                    }
                };
            
                case "moveForward":
                    return new Command() {

                        @Override
                        public void schedule() {
                            m_driveBase.setDriveSpeed(new ChassisSpeeds(0.5, 0.0, 0));
                        }
                        
                        @Override
                        public Set<Subsystem> getRequirements() {
                            Set<Subsystem> ret = new TreeSet<>();
                            ret.add(m_driveBase);
                            return ret;
                        }                        
                    };

                case "PathPlannerTest":
                    return autoBuilder.fullAuto(pathGroup);
                
                default :
                    throw new IllegalArgumentException("Make sure you enter a valid Auton name!"); 
        }
    }

    public Command getAuton() {
        return chooser.getSelected();
    }    
}