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

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileSubsystem;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeArmConstants;

public class IntakeArm extends TrapezoidProfileSubsystem {
  public enum ArmState {OPENED, CLOSED};

  private CANSparkMax m_intakeArm = new CANSparkMax(CAN_IDs.intakeArm_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private final int m_pidSlot = 0;
  private ArmState m_armState = ArmState.CLOSED;
  private TrapezoidProfile.State m_armPosVel = new TrapezoidProfile.State(0, 0);

  private final ArmFeedforward m_feedforward = new ArmFeedforward(
    IntakeArmConstants.kSVolts, IntakeArmConstants.kGVolts,
    IntakeArmConstants.kVVoltSecondPerRad, IntakeArmConstants.kAVoltSecondSquaredPerRad);

  /** Creates a new IntakeArmNew. */
  public IntakeArm() {
    super(
        // The constraints for the generated profiles
        new TrapezoidProfile.Constraints(IntakeArmConstants.kMaxVelocityRadPerSecond, 
        IntakeArmConstants.kMaxAccelerationRadPerSecSquared),
        // The initial position of the mechanism
        IntakeArmConstants.kInitialPositionRad);

    m_intakeArm.restoreFactoryDefaults();
    m_intakeArm.setIdleMode(IdleMode.kBrake);
    m_intakeArm.setSmartCurrentLimit(80);

    // Verify that the soft limit is correct after experimentation 
    m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, IntakeArmConstants.kForwardSoftlimit);
    m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, IntakeArmConstants.kReverseSoftLimit);

    // Set the PID coefficients for the Intake Arm motor
    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_intakeArm.getPIDController();
    m_pidController.setP(IntakeArmConstants.kP, m_pidSlot);
    m_pidController.setI(IntakeArmConstants.kI, m_pidSlot);
    m_pidController.setD(IntakeArmConstants.kD, m_pidSlot);
    m_pidController.setIZone(0, m_pidSlot);
    m_pidController.setFF(0, m_pidSlot);
    m_pidController.setOutputRange(IntakeArmConstants.kMinOutput, IntakeArmConstants.kMaxOutput, m_pidSlot);
  }

  public void openArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.OPENED) {
      return;
    } else {
      setGoal(IntakeArmConstants.kOpenedPosArmRad);
      m_armState = ArmState.OPENED;
    }
  }

  public void closeArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.CLOSED) {
      return;
    } else {
      setGoal(IntakeArmConstants.kClosedPosArmRad);
      m_armState = ArmState.CLOSED;
    }
  }

  public ArmState getArmState() {
    return m_armState;
  }

  public TrapezoidProfile.State getArmPosVel() {
    return m_armPosVel;
  }


  @Override
  protected void useState(TrapezoidProfile.State state) {
    // Use the computed profile state here.
    // Calculate the feedforward from the sepoint
    double feedforward = m_feedforward.calculate(state.position, state.velocity);
    m_armPosVel = state;
    m_pidController.setReference(state.position * IntakeArmConstants.kArmDegToNeoRotConversionFactor, 
      ControlType.kPosition, m_pidSlot, feedforward, ArbFFUnits.kVoltage);
  }
}
