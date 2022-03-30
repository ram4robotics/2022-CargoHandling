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
import frc.robot.Constants.LauncherConstants;

public class Launcher extends SubsystemBase {
  private CANSparkMax m_launcher1 = new CANSparkMax(CAN_IDs.launcher1_ID, MotorType.kBrushless);
  private CANSparkMax m_launcher2 = new CANSparkMax(CAN_IDs.launcher2_ID, MotorType.kBrushless);
  private SparkMaxPIDController m_pidController;
  private RelativeEncoder m_encoder;
  private double m_setpointRPM = LauncherConstants.desiredRPM;

  /** Creates a new Launcher. */
  public Launcher() {
    m_launcher1.restoreFactoryDefaults();
    m_launcher2.restoreFactoryDefaults();
    m_launcher1.setIdleMode(IdleMode.kCoast);
    m_launcher2.setIdleMode(IdleMode.kCoast);
    m_launcher2.follow(m_launcher1, true);

    /**
     * In order to use PID functionality for a controller, a SparkMaxPIDController object
     * is constructed by calling the getPIDController() method on an existing
     * CANSparkMax object
     */
    m_pidController = m_launcher1.getPIDController();

    // Encoder object created to display velocity values
    m_encoder = m_launcher1.getEncoder();

    // set PID coefficients
    m_pidController.setP(LauncherConstants.kP);
    m_pidController.setI(LauncherConstants.kI);
    m_pidController.setD(LauncherConstants.kD);
    m_pidController.setIZone(LauncherConstants.kIz);
    m_pidController.setFF(LauncherConstants.kFF);
    m_pidController.setOutputRange(LauncherConstants.kMinOutput, LauncherConstants.kMaxOutput);

    // display setpoint to dashboard
    SmartDashboard.putNumber("Launcher target RPM", m_setpointRPM);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    m_setpointRPM = SmartDashboard.getNumber("Launcher target RPM", m_setpointRPM);
    SmartDashboard.putNumber("Current RPM", m_encoder.getVelocity());
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void launch() {
    // System.out.print("in Launcher.launch\n");
    // m_launcher1.set(LauncherConstants.launcherMotorSpeed);
    m_pidController.setReference(m_setpointRPM, ControlType.kVelocity);
  }

  public void unclogLauncher() {
    m_launcher1.set(- LauncherConstants.launcherMotorSpeed / 2);
  }

  public void stopLauncher() {
    m_launcher1.set(0.0);
  }
}
