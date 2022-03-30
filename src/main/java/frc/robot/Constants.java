// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;


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
    public final static class EncoderPorts {
        public final static int[] leftEnc = new int[]{2, 3};
        public final static int[] rightEnc = new int[]{4,5};
        public final static int[] intakeArmEnc = new int[]{6,7};
        public final static int[] climberEnc = new int[]{8,9};
    }
    public final class IntakeConstants {
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

        public final static double wheelMotorSpeed = 0.6;
        public final static double armMotorSpeed = 0.8;
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

    public final class OIConstants {
        public final static int xbox1_port = 0;
        public final static int xbox2_port = 1;
    }
}
