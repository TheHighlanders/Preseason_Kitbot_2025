// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Drivetrain extends SubsystemBase {
 
  private final SparkMax FLMotor 
    = new SparkMax(1, MotorType.kBrushed);


  private final SparkMax FRMotor 
    = new SparkMax(2, MotorType.kBrushed);

  private final SparkMax BLMotor 
    = new SparkMax(3, MotorType.kBrushed);
    
  private final SparkMax BRMotor 
    = new SparkMax(4, MotorType.kBrushed);

    private DifferentialDrive drive = new DifferentialDrive(this::left, this::right);
    

  public Drivetrain() {
  }
  public void go(double x, double y) {
    drive.arcadeDrive(x, y);
  }
  

public void left(double left) {
  FLMotor.set(left);
  BLMotor.set(left);
 
}

public void right(double right) {
  FRMotor.set(right);
  BRMotor.set(right);

} 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
