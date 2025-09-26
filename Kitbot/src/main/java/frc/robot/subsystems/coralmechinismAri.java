// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class coralmechinismAri extends SubsystemBase {
  /** Creates a new coralmechinismAri. */ 
  SparkMax motor = new SparkMax(5, MotorType.kBrushed);
  public coralmechinismAri() {}

  public void roll() {
  motor.set(1);
}
public void stopdrop() {
  motor.set(-1);
}
public void grab() {
  motor.set(0);
}
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
