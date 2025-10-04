package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IDs;
import frc.robot.Constants.Configs;
import frc.robot.Constants.Elevator.ElevatorSetpoints;
import frc.robot.Constants.IDs.ElevatorConstants;

public class ElevatorSubsystem extends SubsystemBase {
    public enum Setpoint {
        kFeederStation,
        kLevel1,
        kLevel2,
        kLevel3,
        kLevel4;
    }

    private final SparkMax m_elevatorRight;
    private final SparkMax m_elevatorLeft;
    private final SparkClosedLoopController m_elevatorRightClosedLoopController;
    private final SparkClosedLoopController m_elevatorLeftClosedLoopController;
    private final RelativeEncoder m_elevatorRightEncoder;
    private final RelativeEncoder m_elevatorLeftEncoder;
    private final DigitalInput m_elevatorLimitSwitchTop;
    private final DigitalInput m_elevatorLimitSwitchBottom;


    private boolean b_wasResetByLimitBot;
    private boolean b_wasResetByLimitTop;
    private double elevatorCurrentTarget;

    public ElevatorSubsystem() {
        m_elevatorRight = new SparkMax(IDs.ElevatorConstants.k_elevatorRightID, MotorType.kBrushless);
        m_elevatorLeft = new SparkMax(IDs.ElevatorConstants.k_elevatorLeftID, MotorType.kBrushless);
        m_elevatorRight.configure(Configs.Elevator.elevatorConfigRight, 
                                  ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        m_elevatorLeft.configure(Configs.Elevator.elevatorConfigLeft, 
                                 ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);       
        m_elevatorRightClosedLoopController = m_elevatorRight.getClosedLoopController();
        m_elevatorLeftClosedLoopController = m_elevatorLeft.getClosedLoopController();
        m_elevatorRightEncoder = m_elevatorRight.getEncoder();
        m_elevatorLeftEncoder = m_elevatorLeft.getEncoder();
        resetEncoders();

        m_elevatorLimitSwitchTop = new DigitalInput(ElevatorConstants.k_elevatorSwitchTopID);
        m_elevatorLimitSwitchBottom = new DigitalInput(ElevatorConstants.k_elevatorSwitchBottomID);

        b_wasResetByLimitBot = false;
        b_wasResetByLimitTop = false;
        elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
    }
      
    /**
     * Command to set the subsystem setpoint. This will set the arm and elevator to their predefined
     * positions for the given setpoint.
     */
    public Command setSetpointCommand(Setpoint setpoint) {
        return this.runOnce(
        () -> {
          switch (setpoint) {
            case kFeederStation:
              elevatorCurrentTarget = ElevatorSetpoints.kFeederStation;
              break;
            case kLevel1:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel1;
              break;
            case kLevel2:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel2;
              break;
            case kLevel3:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel3;
              break;
            case kLevel4:
              elevatorCurrentTarget = ElevatorSetpoints.kLevel4;
              break;
          }
        });
    }

    public void testDrive(double speed){
      m_elevatorRight.set(speed);
    }

    /**
     * Drive the arm and elevator motors to their respective setpoints. This will use MAXMotion
     * position control which will allow for a smooth acceleration and deceleration to the mechanisms'
     * setpoints.
     */
    private void moveToSetpoint() {
        //m_elevatorRightClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
        m_elevatorLeftClosedLoopController.setReference(elevatorCurrentTarget, ControlType.kMAXMotionPositionControl);
    }

    /** Zero the elevator encoder when the bottom limit switch is pressed. (Might have to invert limit switch)*/
    private void zeroElevatorOnBottomLimitSwitch() {
        if (!b_wasResetByLimitBot && !m_elevatorLimitSwitchBottom.get()) {
            // Zero the encoder only when the limit switch is switches from "unpressed" to "pressed" to
            // prevent constant zeroing while pressed
            resetEncoders();
            b_wasResetByLimitBot = true;
        } else if (!m_elevatorLimitSwitchBottom.get()) {
            b_wasResetByLimitBot = false;
        }
    }

    /** Set the elevator encoder when the top limit switch is pressed. (Might have to invert limit switch)*/
    private void setElevatorOnTopLimitSwitch() {
      if (!b_wasResetByLimitTop && !m_elevatorLimitSwitchTop.get()) {
          // Set the encoder only when the limit switch is switches from "unpressed" to "pressed" to
          // prevent constant zeroing while pressed
          m_elevatorRightEncoder.setPosition(29);
          m_elevatorLeftEncoder.setPosition(29);
          b_wasResetByLimitTop = true;
      } else if (!m_elevatorLimitSwitchTop.get()) {
          b_wasResetByLimitTop = false;
      }
  }
    
    //Resests elevator encoders to read a position of 0.
    public void resetEncoders() {
        m_elevatorRightEncoder.setPosition(0);
        m_elevatorLeftEncoder.setPosition(0);
    }

    //Stops the elevator's motors
    public void stopMotors() {
        m_elevatorRight.stopMotor();
        m_elevatorLeft.stopMotor();
    }

    @Override
    public void periodic() {
        moveToSetpoint();
        zeroElevatorOnBottomLimitSwitch();
        setElevatorOnTopLimitSwitch();
        
        // Display subsystem values
        SmartDashboard.putNumber("Coral/Elevator/Target Position", elevatorCurrentTarget);
        SmartDashboard.putNumber("Coral/Elevator/Actual Right Position", m_elevatorRightEncoder.getPosition());
        SmartDashboard.putNumber("Coral/Elevator/Actual Left Position", m_elevatorLeftEncoder.getPosition());
        SmartDashboard.putBoolean("Top Limit Switch", !m_elevatorLimitSwitchTop.get());
        SmartDashboard.putBoolean("Bottom Limit Switch", !m_elevatorLimitSwitchBottom.get());
    }
}