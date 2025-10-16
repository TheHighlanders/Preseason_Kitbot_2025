// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftSparkMax = new SparkMax(1, MotorType.kBrushed); //leftleader
  private final SparkMax leftSparkMax2 = new SparkMax(2, MotorType.kBrushed); //leftfollower

  private final SparkMax rightSparkMax = new SparkMax(3, MotorType.kBrushed); //rightleader
  private final SparkMax rightSparkMax2 = new SparkMax(4, MotorType.kBrushed);//rightfollower

  private Timer timer = new Timer();

  private DifferentialDrive dih = new DifferentialDrive(leftSparkMax::set, rightSparkMax::set); 
  //private DifferentialDrive fih = new DifferentialDrive(leftSparkMax2::set, rightSparkMax2::set);

  Field2d field = new Field2d();
  DifferentialDriveOdometry odo = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  // Creates the configuration (aka config) to apply to motors
  double leftIn;
  double rightIn;

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

  public void go(double x, double y) {
    dih.arcadeDrive(-x, -y);
    //fih.arcadeDrive(x, y);

     // Estimate tank x left/right inputs from arcadeDrive values

     double simLeftInput = x + y;
     double simRightInput = x - y;

     


    
    //Drives the robot using arcade x.
   
    double maximum = Math.max(Math.abs(x), Math.abs(y));

    if (x >= 0) {
        if (y >= 0) {
            leftIn = maximum;
            rightIn = simRightInput;
        }
        else {
            leftIn = simLeftInput;
            rightIn = maximum;
        }
      }
    else 
        if (y >= 0) {
            leftIn = simLeftInput;
            rightIn =-maximum;
        }
        else {
            leftIn = -maximum;
            rightIn = simRightInput;
        }
    }

  
  @Override
  public void simulationPeriodic() {
   // Clamp voltages to between -12 and 12
   double leftVoltage = Math.max(-12, Math.min(12, leftIn * 12));
   double rightVoltage = Math.max(-12, Math.min(12, rightIn * 12));

   Util.update(leftVoltage, rightVoltage);

   odo.update(Util.getHeading(), Util.getLeftDistance(), Util.getRightDistance());
   field.setRobotPose(Util.getPose());
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
      new PrintCommand("drove for " + seconds + " sec");
    }); 
  }

    
}

 
