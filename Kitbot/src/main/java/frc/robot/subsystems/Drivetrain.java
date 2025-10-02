// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import java.io.ObjectInputFilter.Config;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Drivetrain extends SubsystemBase {
  private final SparkMax leftSparkMax = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax rightSparkMax = new SparkMax(2, MotorType.kBrushed);
  private final SparkMax leftSparkMax2 = new SparkMax(3, MotorType.kBrushed);
  private final SparkMax rightSparkMax2 = new SparkMax(4, MotorType.kBrushed);


  SparkMaxConfig leftSparkMaxConfig = new SparkMaxConfig();
  SparkMaxConfig rightSparkMaxConfig = new SparkMaxConfig();
  SparkMaxConfig leftSparkMax2Config = new SparkMaxConfig();
  SparkMaxConfig rightSparkMax2Config = new SparkMaxConfig();

  // only need 1
  private DifferentialDrive dih = new DifferentialDrive(leftSparkMax::set, rightSparkMax::set);
  //private DifferentialDrive fih = new DifferentialDrive(leftSparkMax2::set, rightSparkMax2::set);


   double leftIn = 0.0;
   double rightIn = 0.0;

  Field2d field = new Field2d();
  DifferentialDriveOdometry odo = new DifferentialDriveOdometry(new Rotation2d(0), 0, 0);
  
  //bring this back for configs
  public Drivetrain() {
    SmartDashboard.putData("Field", field);
    SparkMaxConfig config = new SparkMaxConfig();
    config.follow(leftSparkMax);
    config.follow(rightSparkMax);
  }

  public void go(double x, double y) {
    dih.arcadeDrive(x, y);
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
  public Command driveautCommand(double xSpeed, double zRotation){
    return runOnce(
    ()-> {
      go(xSpeed, zRotation);
    }); 
  }  
}

 
