// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class drivetrain extends SubsystemBase {
  /** Creates a new drivetrain. */
  public SparkMax motorleftfront = new SparkMax(1, MotorType.kBrushless);
  public SparkMax motorleftback = new SparkMax(2, MotorType.kBrushless);
  public SparkMax motorrightfront = new SparkMax(3, MotorType.kBrushless);
  public SparkMax motorrightback = new SparkMax(4, MotorType.kBrushless);
  motorleftback.follow(motorleftfront)
  motorrightback.follow(motorrightfront)
  m_robotDrive = new DifferentialDrive(motorleftfront::set, motorrightfront;;set)
  
  public drivetrain() {}
  public void driveonce() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
