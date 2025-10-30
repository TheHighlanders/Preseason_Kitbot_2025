package frc.robot;

import choreo.auto.AutoFactory;
import frc.robot.subsystems.CoralReleaser;
import frc.robot.subsystems.Drivetrain;

public class AutoC {
    Drivetrain drivetrain;
    CoralReleaser coralreleaser;
        private AutoFactory autoFactory;
    
        public AutoC(Drivetrain drive, CoralReleaser coral) {
    
    
        autoFactory = new AutoFactory(
         drivetrain::getPose, // A function that returns the current robot pose
         drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
         drivetrain::followTrajectory, // The drive subsystem trajectory follower 
         true, // If alliance flipping should be enabled 
         drivetrain
         ); 
    }
}
