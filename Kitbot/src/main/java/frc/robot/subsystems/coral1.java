// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coral1 extends SubsystemBase {
  /** Creates a new coral1. */
  SparkMax coralmotor = new SparkMax(5, MotorType.kBrushed);
  public coral1() {}
  public void spinout(){
    coralmotor.set(1);
    DriverStation.reportError("spin1",false);
  }
  public void spinin(){
    coralmotor.set(-1);
  }
  public void stop(){
    coralmotor.set(0);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
