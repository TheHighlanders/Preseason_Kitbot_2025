// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

/** Creates a new CoralReleaser subsystem.
   * 
   * 
   */
public class CoralReleaser extends SubsystemBase {
  

  SparkMax Dropper = new SparkMax(5, MotorType.kBrushless);

  public CoralReleaser() {}

  /**
   * Shoots the coral, sets the speed of the motor to 1 
   * 
   * 
   * @param None
   * 
   *
   * 
   */
  public Command RollCMD() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          Dropper.set(1);
          /* one-time action goes here */
        });
  }


  /**
   * Stops the motor, sets the speed to 0
   * @param None
   */
  public Command GrabCMD() {
    
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          Dropper.set(0);/*Goes to downward position NOT FINISHED */
        });
  }

    /**
   * Drives the motor backwards, sets the speed to -1
   * @param None
   */
    public Command StopDropCMD() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          /* one-time action goes here */
          Dropper.set(-1);
        });
  }
  /**
   * An example method querying a boolean state of the subsystem (for example, a digital sensor).
   *
   * @return value of some boolean subsystem state, such as a digital sensor.
   */
  public boolean exampleCondition() {
    // Query some boolean state, such as a digital sensor.
    return false;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
