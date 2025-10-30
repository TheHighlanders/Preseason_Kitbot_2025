package frc.robot;

import frc.robot.subsystems.CoralReleaser;
import frc.robot.subsystems.Drivetrain;

public class AutoC {
Drivetrain drivetrain;
CoralReleaser coralReleaser;

    public AutoC(Drivetrain drive, CoralReleaser coral){
        autoFactory = new AutoFactory(
            driveSubsystem::getPose, // A function that returns the current robot pose
            driveSubsystem::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
            driveSubsystem::followTrajectory, // The drive subsystem trajectory follower 
            true, // If alliance flipping should be enabled 
            driveSubsystem, // The drive subsystem
        );    

    }

}

