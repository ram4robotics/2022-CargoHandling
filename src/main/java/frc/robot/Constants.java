// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
    public final class CAN_IDs {
        public final static int left1_ID = 11;
        public final static int left2_ID = 12;
        public final static int right1_ID = 13;
        public final static int right2_ID = 14;
        public final static int intakeWheels_ID = 21;
        public final static int intakeArm_ID = 22;
        public final static int indexerFront_ID = 23;
        public final static int indexerBack_ID = 24;
        public final static int launcher1_ID = 25;
        public final static int launcher2_ID = 26;
        public final static int climber1_ID = 31;
        public final static int climber2_ID = 32;
    }
    public final static class EncoderConstants {
        public final static int[] leftEnc_ports = new int[]{2, 3};
        public final static boolean leftEnc_reversed = false;
        public final static int[] rightEnc_ports = new int[]{4, 5};
        public final static boolean rightEnc_reversed = true;
        public final static int[] intakeArmEnc_ports = new int[]{6,7};
        public final static boolean intakeArmEnc_reversed = false;
        public final static int[] climberEnc_ports = new int[]{8,9};
        public final static boolean climberEnc_reversed = false;
        public final static double revThroughboreEnc_PPR = 2048;
    }
    public final static class DriveTrainConstants {
        private final static double _gearRatio = 10.71;
        private final static double _wheelDiameter_m = Units.inchesToMeters(6);
        public final static double kWheelCircumference_m = Math.PI * _wheelDiameter_m;
        // ToDo: trackWidth in meters; to be calculated with sysID tool
        private final static double _trackWidth = Units.inchesToMeters(22);
        public static final DifferentialDriveKinematics kDriveKinematics =
            new DifferentialDriveKinematics(_trackWidth);
        // ToDo: These are example values only - DO NOT USE THESE FOR YOUR OWN ROBOT!
        // These characterization values MUST be determined either experimentally or theoretically
        // for *your* robot's drive.
        // The Robot Characterization Toolsuite provides a convenient tool for obtaining these
        // values for your robot.
        public static final double ksVolts = 0.22;
        public static final double kvVoltSecondsPerMeter = 1.98;
        public static final double kaVoltSecondsSquaredPerMeter = 0.2;

        // ToDo: Determine the Feedback constants from sysID tool
        //  These are the values for the SparkMax motor controller
        public static final double kPDriveVel = 8.5;
        public final static double kIDriveVel = 0.0;
        public final static double kDDriveVel = 0.0;

        public final static double kMaxSpeed_m_s = 3.0; // Max velocity 3 meters per second
        public final static double kMaxAccel_m_ss = 3.0; // Max acceleration 3 m/s^2
    }
    public static final class AutoConstants {
        // Reasonable baseline values for a RAMSETE follower in units of meters and seconds
        public static final double kRamseteB = 2;
        public static final double kRamseteZeta = 0.7;
    }
    public final class IntakeWheelConstants {
        // max RPM = Maximum allowed speed for the intake wheels;
        // typically this is less than (motor free load RPM / gear ratio) value.
        public final static double wheelMaxSpeedRPM = 4000; 
        public final static double wheelMaxSpeedDegPerSec = wheelMaxSpeedRPM * 360 / 60;
        // The intake wheels will accelerate from 0 to max speed in one second
        public final static double wheelMaxAccelDPS2 = wheelMaxSpeedDegPerSec;
        // Closed loop velocity PID constants for SparkMax.  
        // These values are copied from the RevRobotics' SparkMax 
        // closed loop velocity example
        // The following constants are taken from the RevRobotics example
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
        public final static double wheelkP = 6e-5; 
        public final static double wheelkI = 0;
        public final static double wheelkD = 0; 
        public final static double wheelkIz = 0;
        public final static double wheelkFF = 0.000015; 
        public final static double kMaxOutput = 1; 
        public final static double kMinOutput = -1;
        public final static double desiredRPM = 3000;
        public final static double maxRPM = 5700;
        // The following fixed speed is applicable only if not using the FeedForward or Feedback controllers
        public final static double wheelMotorSpeed = 0.6;
    }
    public final static class IntakeArmConstants {
        // The following FeedForward estimates are theoritical values calculated from recalc tool (reca.lc)
        // https://www.reca.lc/arm?armMass=%7B%22s%22%3A15%2C%22u%22%3A%22lbs%22%7D&comLength=
        // %7B%22s%22%3A10%2C%22u%22%3A%22in%22%7D&currentLimit=%7B%22s%22%3A60%2C%22u%22%3A
        // %22A%22%7D&efficiency=80&endAngle=%7B%22s%22%3A110%2C%22u%22%3A%22deg%22%7D&
        // iterationLimit=10000&motor=%7B%22quantity%22%3A1%2C%22name%22%3A%22NEO%22%7D&
        // ratio=%7B%22magnitude%22%3A30%2C%22ratioType%22%3A%22Reduction%22%7D&startAngle=
        // %7B%22s%22%3A-20%2C%22u%22%3A%22deg%22%7D
        // Note: 0.3 power was found to be sufficient to move the Arm up. 0.3 power = 12*0.3 = 3.6Volts
        public final static double kGVolts = 1.61; // Volts
        public final static double kSVolts = 3.6-kGVolts; // armkSVolts = Volts_that_take_to_move_the_arm_from_rest - 
                                                                //               armkSVolts
        public final static double kVVoltSecondPerRad = 0.58; // Volts * sec / radians
        public final static double kAVoltSecondSquaredPerRad = 0.07; // Volts * sec^2 / radians

        // The following Feedback estimates are taken from the WPILIB's ArmbotOffboard example
        // PID values for SparkMaxPIDController
        public final static double kP = 0.1;
        public final static double kI = 0;
        public final static double kD = 0;
        // Min and Max output power allowed by SparkMaxPIDController
        public final static double kMinOutput = -0.5;
        public final static double kMaxOutput = 0.75;

        // Arm Positions 
        private final static double _closedPosArmDegrees = 0; // Position of the Arm 1x1 tubing on the robot
        private final static double _openedPosArmDegrees = -125; // Position of the Arm 1x1 tubing on the robot
        public final static double kClosedPosArmRad = Units.degreesToRadians(_closedPosArmDegrees);
        public final static double kOpenedPosArmRad = Units.degreesToRadians(_openedPosArmDegrees);
        private final static double _gearRatio = (4*3*60/24); // 4:1 cartridge + 3:1 cartrdige + 60T:24T sprockets
        // The following two values in terms of Neo motor shaft rotations
        private final static double _closedPosNeoRotations = Units.degreesToRotations(_closedPosArmDegrees * _gearRatio);
        private final static double _openedPosNeoRotations = Units.degreesToRotations(_openedPosArmDegrees * _gearRatio);

        // Arm position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)_closedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)_openedPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations

        // The following are constrains for the TrapezoidalProfile
        // Note that the TrapezoidalProfile takes values in Radians whereas SparkMax's PIDController
        // use number of Shaft Rotations
        private final static double _maxVelocityArmDegPerSec = 60; // => 2 seconds from closed-to-opened position
        private final static double _secondsToPeakVel = 1; // => 1 second to 0-to-peak velocity
        private final static double _maxAccelArmDegPerSecSquared = _maxVelocityArmDegPerSec / _secondsToPeakVel; 
        public final static double kMaxVelocityRadPerSecond = Units.degreesToRadians(_maxVelocityArmDegPerSec);
        public final static double kMaxAccelerationRadPerSecSquared = Units.degreesToRadians(_maxAccelArmDegPerSecSquared);
        public final static double kInitialPositionRad = kClosedPosArmRad;
        public final static double kArmDegToNeoRotConversionFactor = Units.degreesToRotations(_gearRatio);

        // The following fixed speeds are applicable only if not using the FeedForward or Feedback controllers
        public final static double armMotorOpeningSpeed = -0.2;
        public final static double armMotorClosingSpeed = 0.3;
    }
    public final class IndexerConstants {
        public final static double indexerMotorSpeed = 0.5;
        // The following constants are taken from the RevRobotics example
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
        public final static double kP = 6e-5; 
        public final static double kI = 0;
        public final static double kD = 0; 
        public final static double kIz = 0;
        public final static double kFF = 0.000015; 
        public final static double kMaxOutput = 1; 
        public final static double kMinOutput = -1;
        public final static double desiredRPM = 8000;
        public final static double maxRPM = 11000;    
    }

    public final class LauncherConstants {
        public final static double launcherMotorSpeed = 0.9;
        // The following constants are taken from the RevRobotics example
        // https://github.com/REVrobotics/SPARK-MAX-Examples/blob/master/Java/Velocity%20Closed%20Loop%20Control/src/main/java/frc/robot/Robot.java
        public final static double kP = 6e-5; 
        public final static double kI = 0;
        public final static double kD = 0; 
        public final static double kIz = 0;
        public final static double kFF = 0.000015; 
        public final static double kMaxOutput = 1; 
        public final static double kMinOutput = -1;
        public final static double desiredRPM = 4000;
        public final static double maxRPM = 5700;    
    }

    public final static class ClimberElevatorConstants {
        // The following FeedForward estimates are theoritical values calculated from recalc tool (reca.lc)
        // https://www.reca.lc/linear?efficiency=80&load=%7B%22s%22%3A120%2C%22u%22%3A%22lbs%22%7D&
        // motor=%7B%22quantity%22%3A2%2C%22name%22%3A%22NEO%22%7D&ratio=%7B%22magnitude%22%3A20%2C
        // %22ratioType%22%3A%22Reduction%22%7D&spoolDiameter=%7B%22s%22%3A0.787%2C%22u%22%3A%22in
        // %22%7D&travelDistance=%7B%22s%22%3A24%2C%22u%22%3A%22in%22%7D
        // Note: Assuming 0.5 power is sufficient to move the Elevator up. 0.5 power = 12*0.5 = 6Volts
        public final static double kGVolts = 0.38; // Volts
        public final static double kSVolts = 6-kGVolts; // armkSVolts = Volts_that_take_to_move_the_arm_from_rest - 
                                                                //               armkSVolts
        public final static double kVVoltSecondPerMeter = 39; // Volts * sec / radians
        public final static double kAVoltSecondSquaredPerMeter = 0.06; // Volts * sec^2 / radians

        // The following Feedback estimates are taken from the WPILIB's ArmbotOffboard example
        // These are not really applicable to ClimberElevator; they have to be tuned
        // based on experimentation
        // PID values for SparkMaxPIDController
        public final static double kP = 1;
        public final static double kI = 0;
        public final static double kD = 0;
        // Min and Max output power allowed by SparkMaxPIDController
        public final static double kMinOutput = -1;
        public final static double kMaxOutput = 1;

        // Elevator Position
        // Convention: _in=inches, _m=meters
        private final static double _downPos_in = 0;
        private final static double _upPos_in = 20; // ToDo: Verify
        public final static double kDownPos_m = Units.inchesToMeters(_downPos_in);
        public final static double kUpPos_m = Units.inchesToMeters(_upPos_in);
        private final static double _gearRatio = 20; // 5:1 cartridge + 4:1 cartrdige 
        private final static double _winchDia_in = 0.787; // For AndyMark Climber-in-a-box Winch kit 1-stage
        private final static double _winchDia_m = Units.inchesToMeters(_winchDia_in);
        private final static double _winchCircumference_m = _winchDia_m * Math.PI;
        public final static double KElevatorMetersToNeoRotationsFactor = _gearRatio / _winchCircumference_m;
        // The following two values in terms of Neo motor shaft rotations
        private final static double _downPosNeoRotations = KElevatorMetersToNeoRotationsFactor * kDownPos_m;
        private final static double _upPosNeoRotations = KElevatorMetersToNeoRotationsFactor * kUpPos_m;
        // Elevator position limits in units of Neo Motor shaft rotations
        // Add 10% leeway for setting the softlimit
        public final static float kForwardSoftlimit = (float)_downPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations
        public final static float kReverseSoftLimit = (float)_upPosNeoRotations * (float)1.1; // in units of Neo motor shaft rotations

        // The following are constrains for the TrapezoidalProfile
        // Note that the TrapezoidalProfile takes values in Meters whereas SparkMax's PIDController
        // use number of Shaft Rotations
        private final static double _maxVelElevaor_in_p_s = 10; // => 10 inches-per-sec velocity
        private final static double _secondsToPeakVel = 1; // => 1 second to 0-to-peak velocity
        private final static double _maxAccelElevator_in_p_ss = 
            _maxVelElevaor_in_p_s / _secondsToPeakVel;
        public final static double kMaxVelMetersPerSec = Units.inchesToMeters(_maxVelElevaor_in_p_s);
        public final static double kMaxAccelMetersPerSecSquared = Units.inchesToMeters(_maxAccelElevator_in_p_ss);
        public final static double kInitialPosMeters = Units.inchesToMeters(_downPos_in);

        // The following fixed speeds are applicable only if not using the FeedForward or Feedback controllers
        // Note: Elevator uses constant force spring and there is no load so not much power is needed when moving up
        public final static double elevatorMotorUpSpeed = 0.2; 
        public final static double elevatorMotorDownSpeed = -0.75;
    }

    public final class OIConstants {
        public final static int xbox1_port = 0;
        public final static int xbox2_port = 1;
    }
}
