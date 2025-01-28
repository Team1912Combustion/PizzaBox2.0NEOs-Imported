// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.DriverController;
import frc.robot.Constants.OperatorController;
import frc.robot.commands.Autos;
import frc.robot.commands.ZeroHeading;
import frc.robot.subsystems.DriveTrain;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final DriveTrain driveTrain = new DriveTrain();

  private final ZeroHeading zeroHeading = new ZeroHeading(driveTrain);

  // Replace with CommandPS4Controller or CommandJoystick if needed
  //private final CommandXboxController driverController = new CommandXboxController(DriverController.DRIVER_JOYSTICK);
  private final Joystick joystick = new Joystick(0);
  private final CommandXboxController operatorController = new CommandXboxController(OperatorController.OPERATOR_JOYSTICK);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    zeroHeading.addRequirements(driveTrain);

    driveTrain.setDefaultCommand(
      new RunCommand(() -> driveTrain.drive(
        -joystick.getRawAxis(1)*DriveConstants.DRIVE_SPEED, 
        -joystick.getRawAxis(0)*DriveConstants.DRIVE_SPEED, 
        -joystick.getRawAxis(2)*DriveConstants.DRIVE_SPEED, 
        driveTrain.fieldRelative), 
      driveTrain));
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
    JoystickButton zeroHeading = new JoystickButton(joystick, 1);
    zeroHeading.onTrue(new ZeroHeading(driveTrain));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
