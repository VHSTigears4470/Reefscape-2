// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveMotors;
import frc.robot.subsystems.TestMotorsSubsystem;


import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  
  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(0);
  private TestMotorsSubsystem testMotor;

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
     initSubsystems();    
    // Configure the trigger bindings
    configureBindings();
  }

  private void initSubsystems() {
      testMotor = new TestMotorsSubsystem(19, 20);
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
    int preset = 1;
    switch (preset) {
            default:
                    controllerPresetMain();
                    break;
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
        return null;
  }

  // Presets
  /**
   * The default / main preset used for comps
   * Does the following: 
   *    
   */
 
  public void controllerPresetMain() { //subject to change (while/on true)
        double speed = 0.1;
        m_driverController.y().whileTrue(new DriveMotors(testMotor, speed));
        m_driverController.x().whileTrue(new DriveMotors(testMotor, -speed));
      //   m_driverController.y().whileTrue(new DriveMotors(frontRightTurn, speed));
      //   m_driverController.x().whileTrue(new DriveMotors(frontRightTurn, -speed));
        
      //   m_driverController.b().whileTrue(new DriveMotors(rearRightDrive, speed));
      //   m_driverController.a().whileTrue(new DriveMotors(rearRightDrive, -speed));
  }

}
