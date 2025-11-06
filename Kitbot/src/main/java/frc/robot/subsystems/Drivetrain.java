// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter.Config;



import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;

import choreo.trajectory.DifferentialSample;
import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.trajectory.constraint.DifferentialDriveKinematicsConstraint;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftSparkMax = new SparkMax(1, MotorType.kBrushed); //leftleader
  private final SparkMax leftSparkMax2 = new SparkMax(2, MotorType.kBrushed); //leftfollower

  private final SparkMax rightSparkMax = new SparkMax(3, MotorType.kBrushed); //rightleader
  private final SparkMax rightSparkMax2 = new SparkMax(4, MotorType.kBrushed);//rightfollower

  private Timer timer = new Timer();

  private DifferentialDrive dih = new DifferentialDrive(leftSparkMax::set, rightSparkMax::set); 
  //private DifferentialDrive fih = new DifferentialDrive(leftSparkMax2::set, rightSparkMax2::set);

  private final LTVUnicycleController controller = new LTVUnicycleController(0.02);

  Field2d field = new Field2d();
  public DifferentialDriveOdometry odo = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  // Creates the configuration (aka config) to apply to motors
  double leftIn;
  double rightIn;
  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(27));

  
  public Drivetrain() {
    SmartDashboard.putData("Field", field);
    SparkMaxConfig config = new SparkMaxConfig();


     // Sets the config to follow the leftSparkMax then apply it to leftSparkMax2
    // restmode is for if the sparkmax is swapped
    // presist is for in case the sparkmax resets due to a breaker trip
    config.follow(leftSparkMax);
    leftSparkMax2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    config.follow(rightSparkMax);
    rightSparkMax2.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    // removes the following then applies the config to rightSparkMax
    config.disableFollowerMode();
    rightSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    // sets the config to be inverted and applies it to the leftsparkmax
    // this means positive values drive both sides forward
    config.inverted(true);
    leftSparkMax.configure(config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

  }

  public void go(double leftSpeed, double rightSpeed) {
    double maxSpeed = 1;
    leftIn = Math.max(-1, Math.min(1, leftSpeed / maxSpeed));
    rightIn = Math.max(-1, Math.min(1, rightSpeed / maxSpeed));
  }

  /*
   * Commented out because this was preventing the robot from running in the sim
   * The robot should drive in auto now but might struggle in teleop
   */
  // public void go(double x, double y) {
  //   dih.arcadeDrive(-x, -y);
  //   //fih.arcadeDrive(x, y);
  //    // Estimate tank x left/right inputs from arcadeDrive values
  //    double simLeftInput = x + y;
  //    double simRightInput = x - y;
  //   //Drives the robot using arcade x. 
  //   double maximum = Math.max(Math.abs(x), Math.abs(y));
  //   if (x >= 0) {
  //       if (y >= 0) {
  //           leftIn = maximum;
  //           rightIn = simRightInput;
  //       }
  //       else {
  //           leftIn = simLeftInput;
  //           rightIn = maximum;
  //       }
  //     }
  //   else 
  //       if (y >= 0) {
  //           leftIn = simLeftInput;
  //           rightIn =-maximum;
  //       }
  //       else {
  //           leftIn = -maximum;
  //           rightIn = simRightInput;
  //       }
  // }


  /**
   * Drives for x seconds then stops motor
   * 
   * 
   */
  // public Command drive(double seconds, double fwSpeed, double zRot) {
  //   timer.restart();
  //   timer.start();
  //   return runOnce (
  //   () -> {
  //     while (timer.get() < seconds) {
  //       new PrintCommand("" + timer.get());
  //       go(fwSpeed, zRot);
  //     }
  //     go(0, 0);
  //   });   
  // }
  public Command drive(double seconds, double fwd, double rot) {
    return run(() -> go(fwd, rot));
}

  
  public Pose2d getPose() {
    return odo.getPoseMeters();
  }

  public void followTrajectory(DifferentialSample sample) {
    // Get the current pose of the robot
    Pose2d pose = getPose();

    // Get the velocity feedforward specified by the sample
    ChassisSpeeds ff = sample.getChassisSpeeds();

    // Generate the next speeds for the robot
    ChassisSpeeds speeds = controller.calculate(
      pose,
      sample.getPose(),
      ff.vxMetersPerSecond,
      ff.omegaRadiansPerSecond
    );

    DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds); // 
    go(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }

  public void simulationPeriodic() {
    Util.update(leftIn * 12, rightIn * 12);
    odo.update(Util.getHeading(), Util.getLeftDistance(), Util.getRightDistance());
    field.setRobotPose(Util.getPose());
  }

}


 
