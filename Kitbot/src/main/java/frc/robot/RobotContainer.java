// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.coralauto;
import frc.robot.commands.spinning;
import frc.robot.subsystems.CoralReleaser;
import frc.robot.subsystems.Drivetrain;

import java.util.function.DoubleSupplier;

import choreo.auto.AutoFactory;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final CoralReleaser coralreleaser = new CoralReleaser();
  private final Drivetrain drivetrain = new Drivetrain();

   final SendableChooser<Command> m_chooser = new SendableChooser<>();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
      private final AutoFactory autoFactory;
  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    autoFactory = new AutoFactory(
         drivetrain::getPose, // A function that returns the current robot pose
         drivetrain::resetOdometry, // A function that resets the current robot pose to the provided Pose2d
         drivetrain::followTrajectory, // The drive subsystem trajectory follower 
         true, // If alliance flipping should be enabled 
         drivetrain
         ); // The drive subsystem
    // run autos
    final coralauto coralauto = new coralauto(drivetrain, coralreleaser);
    final spinning spinning = new spinning(drivetrain, coralreleaser);
    m_chooser.setDefaultOption("Default Auto", coralauto);
    m_chooser.addOption("My Auto", new spinning(drivetrain,coralreleaser ));
    SmartDashboard.putData("Auto choices", m_chooser);

    new coralauto(drivetrain, coralreleaser);
    // Configure the trigger bindings
    configureBindings();

  }
  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    // shoot the thingy fr
    m_driverController.rightTrigger().onTrue(coralreleaser.RollCMD());

    // stopdrop
    m_driverController.rightBumper().onTrue(coralreleaser.StopDropCMD());

    // grab - passive 
    m_driverController.rightTrigger().onFalse(coralreleaser.GrabCMD());
    m_driverController.rightBumper().onFalse(coralreleaser.GrabCMD());

    // double supplier gets the value whenever you call it, not constant
    DoubleSupplier x = m_driverController::getLeftY;
    DoubleSupplier y = m_driverController::getRightX;
    
    drivetrain.setDefaultCommand(Commands.runOnce(() -> {
      drivetrain.go(y.getAsDouble(), -x.getAsDouble());
    }, drivetrain));
    
  }

  
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return m_chooser.getSelected();
  }
}
