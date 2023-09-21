package frc.robot.auto;

import java.io.File;
import java.io.FileFilter;
import java.util.HashMap;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.FollowPathWithEvents;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import com.pathplanner.lib.server.PathPlannerServer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import frc.robot.Constants;
//import frc.robot.subsystems.superstructure.Superstructure;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.superstructure.ObjectiveTracker.NodeLevel;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.swerve.SwerveConstants;
import frc.robot.util.DriveMotionPlanner;
import frc.robot.util.RobotStateEstimator;

public class AutoRoutineManager {
    private final LoggedDashboardChooser<Command> m_chooser;
    private final HashMap<String, Command> m_eventMap;
    private final HashMap<String, PathPlannerTrajectory> m_trajectoryMap;

    private final Swerve swerve;

    public AutoRoutineManager(Swerve swerve) {
        System.out.println("[Init] Creating Auto Routine Manager");
        m_chooser = new LoggedDashboardChooser<Command>("AutonomousChooser");
        m_eventMap = new HashMap<>();
        m_trajectoryMap = new HashMap<>();

        this.swerve = swerve;


        generateTrajectories();
        generateEventMap();
        generateAutoChoices();
        Logger.getInstance().recordOutput("AutoTraj", new Trajectory());

        if (Constants.kTuningMode) {
            PathPlannerServer.startServer(5811);
        }
    }

  
    private void generateTrajectories() {
        File dir = new File(Filesystem.getDeployDirectory() + "/pathplanner");
        FileFilter filter = new FileFilter() {
            @Override
            public boolean accept(File arg0) {
                return !arg0.isDirectory() && arg0.getName().endsWith(".path");
            }
        };

        File[] files = dir.listFiles(filter);
        for (File file : files) {
            int index = file.getName().lastIndexOf('.');
            String prefix = file.getName().substring(0, index);
            m_trajectoryMap.put(prefix, PathPlanner.loadPath(prefix,
                    SwerveConstants.kMaxAttainableSpeed, SwerveConstants.kMaxAcceleration));
            System.out.println("Generated trajectory: " + prefix);
        }
    }

    private void generateEventMap() {
        m_eventMap.put("intakeCube", new PrintCommand("[Intaking Cube!!!]")
                .andThen(Superstructure.intakeGroundCube().withTimeout(3.0)));
        m_eventMap.put("intakeDown", new PrintCommand("[Intaking Cone!!!]")
                .andThen(Superstructure.intakeGroundCone().withTimeout(3.0)));

        m_eventMap.put("prepScoreHighCone", new PrintCommand("[Scoring High Cone!!!]")
                .andThen(Superstructure.setSuperstructureScore(NodeLevel.HIGH, GamePiece.CONE)));
        m_eventMap.put("prepScoreHighCube", new PrintCommand("[Scoring High Cube!!!]")
                .andThen(Superstructure.setSuperstructureScore(NodeLevel.HIGH, GamePiece.CUBE)));
    }

    private Command getFollowComand(PathPlannerTrajectory path) {
        PPSwerveControllerCommand followCommand =
                new PPSwerveControllerCommand(path, RobotStateEstimator.getInstance()::getPose,
                        DriveMotionPlanner.getForwardController(), // X controller
                        DriveMotionPlanner.getStrafeController(), // Y controller
                        DriveMotionPlanner.getRotationController(), // Rotation controller
                        swerve::driveOpenLoop, // Module states consumer
                        true, // Should the path be mirrored depending on alliance color.
                        swerve // Requires swerve subsystem
                );
        return new FollowPathWithEvents(followCommand, path.getMarkers(), m_eventMap).alongWith(
                new InstantCommand(() -> Logger.getInstance().recordOutput("FollowPath", path)));
    }

    public Command getAutoCommand() {
        return m_chooser.get();
    }

    private void setPose(Pose2d pose) {
        RobotStateEstimator.getInstance().setPose(pose);
    }

}
