/** 
// TODO Fix Superstructure
package frc.robot.subsystems.superstructure;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.RobotContainer;
import frc.robot.subsystems.roller.Roller;
import frc.robot.subsystems.superstructure.ObjectiveTracker.GamePiece;
import frc.robot.subsystems.superstructure.ObjectiveTracker.NodeLevel;
import frc.robot.subsystems.wrist.Wrist;

public class Superstructure {
    private static final Wrist wrist = RobotContainer.m_wrist;
    private static final Roller roller = RobotContainer.m_roller;

    public static Command setSuperstructureGoal(SuperstructureGoal goal) {
       /

    public static Command setSuperstructureScore(NodeLevel level, GamePiece piece) {
        SuperstructureGoal goal;
        if (piece == GamePiece.CUBE) {
            switch (level) {
                case HYBRID:
                    goal = SuperstructureGoal.L1_SCORE;
                case MID:
                    goal = SuperstructureGoal.L2_CUBE;
                case HIGH:
                    goal = SuperstructureGoal.L3_CUBE;
                default:
                    goal = SuperstructureGoal.L1_SCORE;
            }
        } else {
            switch (level) {
                case HYBRID:
                    goal = SuperstructureGoal.L1_SCORE;
                case MID:
                    goal = SuperstructureGoal.L2_CONE;
                case HIGH:
                    goal = SuperstructureGoal.L3_CONE;
                default:
                    goal = SuperstructureGoal.L1_SCORE;
            }
        }
        return setSuperstructureGoal(goal);
    }

    public static Command setScoreTeleop() {
        return setSuperstructureScore(ObjectiveTracker.getNodeLevel(),
                ObjectiveTracker.getGamePiece())
                        .finallyDo(interupted -> setSuperstructureGoal(SuperstructureGoal.STOW));
    }

    public static Command scoreCubeLevel(NodeLevel level) {
        return Commands.sequence(setSuperstructureScore(level, GamePiece.CUBE),
                setSuperstructureGoal(SuperstructureGoal.STOW));
    }

    public static Command scoreConeLevel(NodeLevel level) {
        return Commands.sequence(setSuperstructureScore(level, GamePiece.CONE),
             //   setSuperstructureGoal(SuperstructureGoal.STOW));
    }


    public static class SuperstructureGoal {

        public static final SuperstructureGoal STOW = new SuperstructureGoal(-9.3, 0.0);

        public static final SuperstructureGoal GROUND_CONE_INTAKE =
                new SuperstructureGoal(-9.3, 0.0);
        public static final SuperstructureGoal GROUND_CUBE_INTAKE =
                new SuperstructureGoal(-9.3, 0.0);

        public static final SuperstructureGoal SLIDE_CUBE_INTAKE =
                new SuperstructureGoal(50.0, 0.00);

        public static final SuperstructureGoal SCORE_STANDBY = new SuperstructureGoal(39.8, 0.0);

        public static final SuperstructureGoal L1_SCORE = new SuperstructureGoal(15.0, 0.0);

        public static final SuperstructureGoal L2_CONE = new SuperstructureGoal(39.8, 0.556);
        public static final SuperstructureGoal L2_CUBE = new SuperstructureGoal(39.8, 0.556);

        public static final SuperstructureGoal L3_CONE = new SuperstructureGoal(39.8, 1.04);
        public static final SuperstructureGoal L3_CUBE = new SuperstructureGoal(39.8, 1.04);

        public double wrist; // degrees

        public SuperstructureGoal(double elevator, double wrist) {
            this.wrist = wrist;
        }

        // Default Constructor
        public SuperstructureGoal() {
            this(0, 0);
        }

    }
} */