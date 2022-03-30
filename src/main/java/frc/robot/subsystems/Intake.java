// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  public enum ArmState {OPENED, CLOSED};

  private CANSparkMax m_intakeWheels = new CANSparkMax(CAN_IDs.intakeWheels_ID, MotorType.kBrushless);
  private CANSparkMax m_intakeArm = new CANSparkMax(CAN_IDs.intakeArm_ID, MotorType.kBrushless);
  private ArmState m_armState = ArmState.CLOSED;

  private SparkMaxPIDController m_wheelPIDController;
  private RelativeEncoder m_wheelEncoder;
  private double m_wheelSetpointRPM = 4000;
  
  /** Creates a new Intake. */
  public Intake() {
    m_intakeWheels.restoreFactoryDefaults();
    m_intakeArm.restoreFactoryDefaults();
    m_intakeWheels.setIdleMode(IdleMode.kCoast);
    m_intakeArm.setIdleMode(IdleMode.kCoast);

    m_intakeWheels.setSmartCurrentLimit(30);
    m_intakeArm.setSmartCurrentLimit(80);

    // ToDo:  Verify that the soft limit is correct after experimentation 
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kForward, true);
    // m_intakeArm.enableSoftLimit(SoftLimitDirection.kReverse, true);
    // m_intakeArm.setSoftLimit(SoftLimitDirection.kForward, 20);
    // m_intakeArm.setSoftLimit(SoftLimitDirection.kReverse, 0);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_wheelPIDController = m_intakeWheels.getPIDController();

    // Encoder object created to display velocity values
    m_wheelEncoder = m_intakeWheels.getEncoder();

    // set PID coefficients
    m_wheelPIDController.setP(IntakeConstants.wheelkP);
    m_wheelPIDController.setI(IntakeConstants.wheelkI);
    m_wheelPIDController.setD(IntakeConstants.wheelkD);
    m_wheelPIDController.setIZone(IntakeConstants.wheelkIz);
    m_wheelPIDController.setFF(IntakeConstants.wheelkFF);
    m_wheelPIDController.setOutputRange(IntakeConstants.kMinOutput, IntakeConstants.kMaxOutput);

    // display setpoint to dashboard
    SmartDashboard.putNumber("Intake Wheels target RPM", m_wheelSetpointRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_wheelSetpointRPM = SmartDashboard.getNumber("Intake Wheels target RPM", m_wheelSetpointRPM);
    SmartDashboard.putNumber("Intake Wheels Current RPM", m_wheelEncoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void openArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.OPENED) {
      return;
    }

    // ToDo: Apply power until the arm is opened

    // ToDo: Set the state to OPENED
  }

  public void closeArm() {
    // If the arm already opened, nothing more to do.
    if (m_armState == ArmState.CLOSED) {
      return;
    }

    // ToDo: Apply power until the arm is closed

    // ToDo: Set the state to CLOSED
  }

  public ArmState getArmState() {
    return m_armState;
  }

  public void pullInCargo() {
    // m_intakeWheels.set(IntakeConstants.wheelMotorSpeed);
    m_wheelPIDController.setReference(m_wheelSetpointRPM, ControlType.kVelocity);
  }

  public void pushOutCargo() {
    m_intakeWheels.set(-IntakeConstants.wheelMotorSpeed);
  }

  public void stopIntakeWheels() {
    m_intakeWheels.set(0.0);
  }
}
