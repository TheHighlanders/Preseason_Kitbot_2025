// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class EpicThing extends SubsystemBase {
  /** Creates a new EpicThing. */
  public final SparkMax TheSparkMaxThing = new SparkMax(5, MotorType.kBrushed);
  public EpicThing() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

  }

  public Command EpicThingStart() {
    return runOnce(
      () -> {
        TheSparkMaxThing.set(1);
      }
    );
  }

  public Command EpicThingStop() {
    return runOnce(
      () -> {
        TheSparkMaxThing.set(0);
      }
    );
  }
}
