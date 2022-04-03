// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.ClimberElevatorConstants;

public class ClimberElevator extends TrapezoidProfileSubsystem {
  public enum ElevatorState {UP, DOWN};

  private CANSparkMax m_elevator1 = new CANSparkMax(CAN_IDs.climber1_ID, MotorType.kBrushless);
  private CANSparkMax m_elevator2 = new CANSparkMax(CAN_IDs.climber2_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private final int m_pidSlot = 0;
  private ElevatorState m_elevatorState = ElevatorState.DOWN;
  private TrapezoidProfile.State m_elevatorPosVel = new TrapezoidProfile.State(0, 0);

  private final ElevatorFeedforward m_feedforward = new ElevatorFeedforward(
    ClimberElevatorConstants.kSVolts, ClimberElevatorConstants.kGVolts, 
    ClimberElevatorConstants.kVVoltSecondPerMeter, ClimberElevatorConstants.kAVoltSecondSquaredPerMeter);

  /** Creates a new ClimberElevator. */
  public ClimberElevator() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(ClimberElevatorConstants.kMaxVelMetersPerSec, 
              ClimberElevatorConstants.kMaxAccelMetersPerSecSquared),
        // The initial position of the mechanism
        ClimberElevatorConstants.kInitialPosMeters);
    m_elevator1.restoreFactoryDefaults();
    m_elevator1.setIdleMode(IdleMode.kBrake);
    m_elevator1.setSmartCurrentLimit(80);

    m_elevator2.restoreFactoryDefaults();
    m_elevator2.setIdleMode(IdleMode.kBrake);
    m_elevator2.setSmartCurrentLimit(80);

    m_elevator2.follow(m_elevator1, true);

    m_elevator1.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_elevator1.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_elevator1.setSoftLimit(SoftLimitDirection.kForward, ClimberElevatorConstants.kForwardSoftlimit);
    m_elevator1.setSoftLimit(SoftLimitDirection.kReverse, ClimberElevatorConstants.kReverseSoftLimit);

    // Set the PID coefficients for the Elevator motor
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_elevator1.getPIDController();
    m_pidController.setP(ClimberElevatorConstants.kP, m_pidSlot);
    m_pidController.setI(ClimberElevatorConstants.kI, m_pidSlot);
    m_pidController.setD(ClimberElevatorConstants.kD, m_pidSlot);
    m_pidController.setIZone(0, m_pidSlot);
    m_pidController.setFF(0, m_pidSlot);
    m_pidController.setOutputRange(ClimberElevatorConstants.kMinOutput, ClimberElevatorConstants.kMaxOutput, m_pidSlot);

  }

  public void raiseClimberElevator() {
    if (m_elevatorState == ElevatorState.UP) {
      return;
    } else {
      setGoal(ClimberElevatorConstants.kUpPos_m);
      m_elevatorState = ElevatorState.UP;
    }
  }

  public void lowerClimberElevator() {
    if (m_elevatorState == ElevatorState.DOWN) {
      return;
    } else {
      setGoal(ClimberElevatorConstants.kDownPos_m);
      m_elevatorState = ElevatorState.DOWN;
    }
  }

  public ElevatorState getClimberElevatorState() {
    return m_elevatorState;
  }

  public TrapezoidProfile.State getClimberElevatorPosVel() {
    return m_elevatorPosVel;
  }

  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(state.position, state.velocity);
    m_elevatorPosVel = state;
    m_pidController.setReference(state.position * ClimberElevatorConstants.KElevatorMetersToNeoRotationsFactor, 
        ControlType.kPosition, m_pidSlot, feedforward, ArbFFUnits.kVoltage);
  }
}
