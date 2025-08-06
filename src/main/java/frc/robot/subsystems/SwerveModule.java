// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.Drive.Constants.MotorLocation;
import frc.robot.Constants.Configs;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final SparkMax m_driveMotor;
  private final SparkMax m_turnMotor;

  private final RelativeEncoder m_driveEncoder;
  private final AbsoluteEncoder m_turnEncoder;

  private final SparkClosedLoopController m_driveController;
  private final SparkClosedLoopController m_turnController;

  private final MotorLocation m_motorLocation;
  private final double m_driveEncoderInverted;

  private double m_chassisAngularOffset = 0; //update(?)
  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());
 
  /*Constructs a REV MAXSwerve Module designed with NEOs, SPARKS MAX, 
    and a Through Bore Encoder.*/
  public SwerveModule(int p_driveID, int p_turnID, double p_offset, boolean p_inverted, SparkMaxConfig p_config, MotorLocation p_location) {
    m_driveMotor = new SparkMax(p_driveID, MotorType.kBrushless);
    m_turnMotor = new SparkMax(p_turnID, MotorType.kBrushless);

    m_driveEncoder = m_driveMotor.getEncoder();
    m_turnEncoder = m_turnMotor.getAbsoluteEncoder();

    m_driveController = m_driveMotor.getClosedLoopController();
    m_turnController = m_turnMotor.getClosedLoopController();

    m_motorLocation = p_location;

    /*Reset to default configs before applying our own, persisting 
      them to last between power cycles.*/
    m_driveMotor.configure(p_config, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    m_turnMotor.configure(Configs.SwerveModule.k_turningConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters); //Add turn config constant

    m_chassisAngularOffset = p_offset;
    m_desiredState.angle = new Rotation2d(m_turnEncoder.getPosition());
    if(p_inverted)
      m_driveEncoderInverted = -1.0;
    else 
      m_driveEncoderInverted = 1.0;
    m_driveEncoder.setPosition(0);
  }

  public void setDesiredState(SwerveModuleState p_desiredState) {
    //Apply chassis offset to the desired state.
    SwerveModuleState correctedDesiredState = new SwerveModuleState();
    correctedDesiredState.speedMetersPerSecond = p_desiredState.speedMetersPerSecond;
    correctedDesiredState.angle = p_desiredState.angle.plus(Rotation2d.fromRadians(m_chassisAngularOffset));

    //Optimize the reference state as to not turn more than 90 degrees.
    correctedDesiredState.optimize(new Rotation2d(m_turnEncoder.getPosition()));

    //Command driving and turning SPARKS toward their respective setpoints.
    m_driveController.setReference(correctedDesiredState.speedMetersPerSecond, ControlType.kVelocity);
    m_turnController.setReference(correctedDesiredState.angle.getRadians(), ControlType.kPosition);

    m_desiredState = p_desiredState;
  }


  //Returns the current module state.
  public SwerveModuleState getState() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModuleState(
      getDriveVelocity(),
      new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  //Returns the current position of the module.
  public SwerveModulePosition getPosition() {
    //Apply chassis offset to the encoder position to get the position relative to the chassis.
    return new SwerveModulePosition(
        getDrivePosition(),
        new Rotation2d(m_turnEncoder.getPosition() - m_chassisAngularOffset));
  }

  //Returns the encoder position of the drive motor in radians.
  public double getDrivePosition() {
    return m_driveEncoderInverted * m_driveEncoder.getPosition();
  }

  //Returns the module's drive velocity in m/s.
  public double getDriveVelocity() {
    return m_driveEncoder.getVelocity();
  }

  //Returns the position of the turn motor in radians.
  public double getTurnPosition() {
    return m_turnEncoder.getPosition();
  }

  //Returns the module's turn velocity in m/s.
  public double getTurnVelocity() {
    return m_turnEncoder.getVelocity();
  }

  //Zeroes the module's encoders.
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
  }

  //Stops the module's motors;
  public void stopMotors() {
    m_driveMotor.stopMotor();
    m_turnMotor.stopMotor();
  }

  //Tests motor speed or turns it to a set angle (radians)
  public void testDriveMotors(double p_speed) {
    m_driveMotor.set(p_speed);
  }
  public void testTurnMotors(double p_position) {
    m_turnController.setReference(p_position, ControlType.kPosition);
  }

  public void updateSmartDashboard() {
    SmartDashboard.putNumber(m_motorLocation + " driver enoder", getDrivePosition());
    SmartDashboard.putNumber(m_motorLocation + " driver velocity", getDriveVelocity());
    SmartDashboard.putNumber(m_motorLocation + " turn encoder", m_turnEncoder.getPosition());
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
