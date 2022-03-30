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
import frc.robot.Constants.IndexerConstants;

public class Indexer extends SubsystemBase {
  private CANSparkMax m_indexerFront = new CANSparkMax(CAN_IDs.indexerFront_ID, MotorType.kBrushless);
  private CANSparkMax m_indexerBack = new CANSparkMax(CAN_IDs.indexerBack_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double m_setpointRPM = IndexerConstants.desiredRPM;
  /** Creates a new Indexer. */
  public Indexer() {
    m_indexerFront.restoreFactoryDefaults();
    m_indexerBack.restoreFactoryDefaults();
    m_indexerFront.setIdleMode(IdleMode.kBrake);
    m_indexerBack.setIdleMode(IdleMode.kBrake);
    // ToDo:  Check if the smart current limit is too low
    m_indexerFront.setSmartCurrentLimit(30);
    m_indexerBack.setSmartCurrentLimit(30);
    m_indexerBack.follow(m_indexerFront, true);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_indexerFront.getPIDController();

    // Encoder object created to display velocity values
    m_encoder = m_indexerFront.getEncoder();

    // set PID coefficients
    m_pidController.setP(IndexerConstants.kP);
    m_pidController.setI(IndexerConstants.kI);
    m_pidController.setD(IndexerConstants.kD);
    m_pidController.setIZone(IndexerConstants.kIz);
    m_pidController.setFF(IndexerConstants.kFF);
    m_pidController.setOutputRange(IndexerConstants.kMinOutput, IndexerConstants.kMaxOutput);

    // display setpoint to dashboard
    SmartDashboard.putNumber("Indexer target RPM", m_setpointRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_setpointRPM = SmartDashboard.getNumber("Indexer target RPM", m_setpointRPM);
    SmartDashboard.putNumber("Current RPM", m_encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void feedToLauncher() {
    System.out.print("in feedToLauncher\n");
    // m_indexerFront.set(IndexerConstants.indexerMotorSpeed);
    m_pidController.setReference(m_setpointRPM, ControlType.kVelocity);
  }

  public void throwAwayToIntake() {
    System.out.print("In throwAwayToIntake\n");
    m_indexerFront.set(-IndexerConstants.indexerMotorSpeed);
  }

  public void stopIndexer() {
    m_indexerFront.set(0.0);
  }
}
