package frc.robot;

import frc.robot.subsystems.CoralReleaser;
import frc.robot.subsystems.Drivetrain;

public class AutoC {
    Drivetrain drivetrain
    public void AutoC() {
        autoFactory = new AutoFactory(
            drivetrain::getPose, // A function that returns the current robot pose
            drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
          drivetrain::followTrajectory, // The Drive Subsystem trajectory follower
      true, //If allience flipping should be enabled
      drivetrain, // The Drive Subsystem
          );
    }
    public AutoRoutine pickupAndScoreAuto() {
        AutoRoutine routine = autoFactory.newRoutine("taxi");
    
        // Load the routine's trajectories
        AutoTrajectory driveToMiddle = routine.trajectory("driveToMiddle");
    
        // When the routine begins, reset odometry and start the first trajectory (1)
        routine.active().onTrue(
            Commands.sequence(
                driveToMiddle.resetOdometry(),
                driveToMiddle.cmd()
            )
        );
    
        return routine;
    }
    
}