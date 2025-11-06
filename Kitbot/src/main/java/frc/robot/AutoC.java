package frc.robot;

import choreo.auto.AutoFactory;
import choreo.auto.AutoRoutine;
import choreo.auto.AutoTrajectory;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Drivetrain;

public class AutoC {
    Drivetrain drivetrain;
    AutoFactory autoFactory;
    public void AutoC(){
        autoFactory = new AutoFactory(
        drivetrain::getPose, // A function that returns the current robot pose
        drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
        drivetrain::followTrajectory, // The Drive Subsystem trajectory follower
        true, //If allience flipping should be enabled
    drivetrain, // The Drive Subsystem
        
        );
    }

    public AutoRoutine pickupAndScoreAuto() {
    AutoRoutine routine = autoFactory.newRoutine("Taxi");

    AutoTrajectory driveToMiddle = routine.trajectory("driveToMiddle")

    routine.active().onTrue(
    Commands.sequence(
            driveToMiddle.resetOdometry(),
            driveToMiddle.cmd()
          )
      );

       return routine;
    }
}
