// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.CAN_IDs;
import frc.robot.Constants.IntakeWheelConstants;

public class IntakeWheels extends SubsystemBase {
  private CANSparkMax m_intakeWheels = new CANSparkMax(CAN_IDs.intakeWheels_ID, MotorType.kBrushless);

  private SparkMaxPIDController m_wheelPIDController;
  private RelativeEncoder m_wheelEncoder;
  private double m_wheelSetpointRPM = 4000;
  
  /** Creates a new Intake. */
  public IntakeWheels() {
    m_intakeWheels.restoreFactoryDefaults();
    m_intakeWheels.setIdleMode(IdleMode.kCoast);

    m_intakeWheels.setSmartCurrentLimit(30);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_wheelPIDController = m_intakeWheels.getPIDController();

    // Encoder object created to display velocity values
    m_wheelEncoder = m_intakeWheels.getEncoder();

    // set PID coefficients
    m_wheelPIDController.setP(IntakeWheelConstants.wheelkP);
    m_wheelPIDController.setI(IntakeWheelConstants.wheelkI);
    m_wheelPIDController.setD(IntakeWheelConstants.wheelkD);
    m_wheelPIDController.setIZone(IntakeWheelConstants.wheelkIz);
    m_wheelPIDController.setFF(IntakeWheelConstants.wheelkFF);
    m_wheelPIDController.setOutputRange(IntakeWheelConstants.kMinOutput, IntakeWheelConstants.kMaxOutput);

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

  public void pullInCargo() {
    // m_intakeWheels.set(IntakeConstants.wheelMotorSpeed);
    m_wheelPIDController.setReference(m_wheelSetpointRPM, ControlType.kVelocity);
  }

  public void pushOutCargo() {
    m_intakeWheels.set(-IntakeWheelConstants.wheelMotorSpeed);
  }

  public void stopIntakeWheels() {
    m_intakeWheels.set(0.0);
  }
}
