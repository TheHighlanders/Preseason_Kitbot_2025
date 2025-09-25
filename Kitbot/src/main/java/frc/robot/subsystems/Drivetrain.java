// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class ExampleSubsystem extends SubsystemBase {
  private final SparkMax leftSparkMax = new SparkMax(1, MotorType.kBrushed);
  private final SparkMax rightSparkMax = new SparkMax(2, MotorType.kBrushed);
  //private final SparkMax leftSparkMax = new SparkMax(3, MotorType.kBrushed);
  //private final SparkMax rightSparkMax = new SparkMax(deviceId:4, MotorType.kBrushed);
}
  double speed = 0.7;
  
  
  //forward
  public Command forwardCMD() {

    return Commands.runOnce(() -> {
      rightSparkMax.set(speed);
      leftSparkMax.set(speed);
    });
  }

    public Command forward() { // back

      return Commands.runOnce(() ->{
      rightSparkMax.set(-speed);
      leftSparkMax.set(-speed);
    });

    public Command.runOnce(()-> {

      return Command.runOnce(() ->{  //turn left
        rightSparkMax.set(speed);
      });
  
  

  //backward



  //right




  //left
  




  }
}
