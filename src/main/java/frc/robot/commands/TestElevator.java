// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.ElevatorSubsystem;
import edu.wpi.first.wpilibj2.command.Command;

/** An example command that uses an example subsystem. */
public class TestElevator extends Command {
  private final ElevatorSubsystem m_eleavtorSub;
  private final double d_speed;

  public TestElevator(ElevatorSubsystem p_subsystem, double p_speed) {
    m_eleavtorSub = p_subsystem;
    d_speed = p_speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(p_subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_eleavtorSub.testDrive(d_speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_eleavtorSub.stopMotors();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
