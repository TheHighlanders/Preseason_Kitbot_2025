// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import static edu.wpi.first.units.Units.Newton;

import com.fasterxml.jackson.databind.type.ResolvedRecursiveType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.controller.LTVUnicycleController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.kinematics.Kinematics;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.AnalogGyro;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftSparkMax = new SparkMax(1, MotorType.kBrushed); //leftleader
  private final SparkMax leftSparkMax2 = new SparkMax(2, MotorType.kBrushed); //leftfollower

  private final SparkMax rightSparkMax = new SparkMax(3, MotorType.kBrushed); //rightleader
  private final SparkMax rightSparkMax2 = new SparkMax(4, MotorType.kBrushed);//rightfollower
  private final LTVUnicycleController controller = new LTVUnicycleController(0.02);
   private Timer timer = new Timer();
   
  private final AnalogGyro gyro = new AnalogGyro(0);
private final Encoder leftencoder = new Encoder(0,1);
private final Encoder rightencoder = new Encoder(2,3);
 

  private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(Units.inchesToMeters(0));
  private DifferentialDrive dih = new DifferentialDrive(leftSparkMax::set, rightSparkMax::set); 
  private final DifferentialDriveOdometry odo = new DifferentialDriveOdometry(null, null, null);
  //private DifferentialDrive fih = new DifferentialDrive(leftSparkMax2::set, rightSparkMax2::set);
// we miss fih <\3
  public Drivetrain() {
    // Creates the configuration (aka config) to apply to motors
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
  
  public void go(double x, double y) {
    dih.arcadeDrive(x, y);
  }
  /**
   * Drives for x seconds then stops motor
   * 
   * 
   */
  public Command drive(double seconds, double fwSpeed, double zRot) {
    timer.restart();
    return runOnce (
    () -> {
      while (timer.get() < seconds) {
        go(fwSpeed, zRot);
      }
      go(0, 0);
    }); 
  }
public void resetOdometry(Pose2d pose){
  odo.resetPosition(gyro.getRotation2d(), leftencoder.getDecodingScaleFactor(), rightencoder.getDistance(),pose);
}
    public Command auto(double x,double y){
    return runOnce(
    ()-> {
      go(x, y);
    }); 
  }  
  public Pose2d getPose(){
  return odo.getPoseMeters();
  }
  public void followTrajectory (DifferentialSample sample){
      // Get the current pose of the robot
        Pose2d pose = getPose();

        // Get the velocity feedforward specified by the sample
        ChassisSpeeds ff = sample.getChassisSpeeds();
        //copy and paste 4 the win!
 
         // Generate the next speeds for the robot
         ChassisSpeeds speeds = controller.calculate(
             pose,
             sample.getPose(),
             ff.vxMetersPerSecond,
             ff.omegaRadiansPerSecond
         );
           DifferentialDriveWheelSpeeds wheelSpeeds = kinematics.toWheelSpeeds(speeds);
        go(wheelSpeeds.leftMetersPerSecond, wheelSpeeds.rightMetersPerSecond);
  }
  
}