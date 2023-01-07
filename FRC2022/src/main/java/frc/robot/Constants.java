// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix.motorcontrol.TalonFXInvertType;

import org.opencv.core.Mat;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.util.Color;
import frc.robot.utilities.PIDGains;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

    public static class Climber {
        public static final double DEGREES_TO_ROTATIONS_CONVERSION_FACTOR = 0.0;
        public static final int LEFT_LONG_HOOK_MOTOR_PORT = 28;
        public static final int LEFT_SHORT_HOOK_MOTOR_PORT = 29;
        public static final int RIGHT_SHORT_HOOK_MOTOR_PORT = 26;
        public static final int RIGHT_LONG_HOOK_MOTOR_PORT = 27;
        public static final int MASTER_MOTOR_PORT = 18;
        public static final int SLAVE_MOTOR_PORT = 19;

        public static final PIDGains SHOULDER_MOTOR_PID_GAINS = new PIDGains(0.05, 0.0, 0.0, 0.011, 0.0, 0.0, -1.0, 1.0,
                "Climber/Shoulder Gains");
        public static final double DEGREE_TO_ROTATIONS_SHOULDER_CONV_FACTOR = 0;
        public static final double SENSOR_UNITS_TO_DEGREES = 0;
        public static final boolean kSensorPhase = false;
        public static final TalonFXInvertType kMotorInvert = null;
        // public static final double HOOK_DEGREES_TO_ROTATIONS_CONVERSION_FACTOR = 0;
        public static final double POSITION_TOLERANCE = 0;
        public static final int RIGHT_LONG_LIMIT_SWITCH = 2;
        public static final int RIGHT_SHORT_LIMIT_SWITCH = 3;
        public static final int LEFT_SHORT_LIMIT_SWITCH = 1;
        public static final int LEFT_LONG_LIMIT_SWITCH = 0;
        public static final double SHOULER_SPEED = -1.0;
        public static final double SHOULDER_ANGLE_TOLERANCE = 3.0; // Degrees
        public static final double HOOK_POSITION_CONVERSION_FACTOR = 180.0 / 28.0; // 1.0 / (1.0 / 12.0 * 18.0 / 84.0 *
                                                                                   // 180.0 / Math.PI);
                                                                                   // //1/12*18/84*360;
        public static int kPIDLoopIdx = 0;
        public static int kTimeoutMs = 0;
        public static final PIDGains HOOK_MOTOR_PID_GAINS = new PIDGains(0.06, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0,
                "Hook");
        public static final double HOOK_ANGLE_TOLERANCE = 2.0; // Degrees
        public static final double DEFAULT_HOOK_ALPHA = 0.1;
        public static final int CANCODER_PORT = 5;
        public static final double CANCODER_ABSOLUTE_POSITION_CONVERSION_FACTOR = 1.0 / 5.0;
        public static final double MINIMUM_STARTING_ANGLE = -66.0;
        public static final double MAXIMUM_STARTING_ANGLE = 6.0;
        public static final double kS = 0.01;
        public static final double kv = 0.01;
        public static final double SHOULDER_TALON_POSITION_FACTOR = 2048.0 * 1.25;// 0.75;//0.7125;//0.782;//1.25 * 20.0 / 12.0;/*
                                                                                //    * 1.0 / 583.333 * 180.0 / Math.PI *
                                                                                //    */ /* 2048.0 * (459.5 / 360); */

        // public static final double SHOULDER_TALON_POSITION_FACTOR = 4.667 * 25.0;
        public static final double CANCODER_ABSOLUTE_POSITION_OFFSET = 70.4;// -165.0; // degrees, this is the
                                                                              // absolute encoder reading for where the
                                                                              // climber arm should idealy be in
                                                                              // starting config
        public static final double CANCODER_TO_OUTPUT_STAGE = 1.0 / 5.0; // For every 5 rotations, the shoulder will
                                                                         // rotate once
        public static final double SHOULDER_STARTING_POSITION = -23.6; // 0 degrees is horizontal, clockwise (looking
                                                                       // from the side with intake pointing left) is
                                                                       // positive, so starting pos will be negative

        public static final double LEFT_LONG_CLOSED_POSITION = 1.0;
        public static final double LEFT_SHORT_CLOSED_POSITION = 1.0;
        public static final double RIGHT_LONG_CLOSED_POSITION = 1.0;
        public static final double RIGHT_SHORT_CLOSED_POSITION = 1.0;

        public static final double LEFT_LONG_OPEN_POSITION = 200.0;
        public static final double LEFT_SHORT_OPEN_POSITION = 200.0;
        public static final double RIGHT_LONG_OPEN_POSITION = 200.0;
        public static final double RIGHT_SHORT_OPEN_POSITION = 200.0;

        public static final double LONG_SIDE_RELEASE_POSITION = 195.0;
        public static final double SHORT_SIDE_RELEASE_POSITION = 195.0;
        public static final double SHORT_SIDE_ACCEPTING_POSITION = 65.0;
        public static final double LONG_SIDE_ACCEPTING_POSITION = 65.0;

    }

    public static class Tower {

        public static final int TOWER_CURRENT_LIMIT = 0;
        public static final double TOWER_FORWARDS_SPEED = 1.0;
        public static final double TOWER_BACKWARDS_SPEED = -1.0 * TOWER_FORWARDS_SPEED;
        public static final double AUTO_TOWER_BACKWARDS_SPEED = -0.50 * TOWER_FORWARDS_SPEED;
        public static final int BALL_DETECTING_DISTANCE = 230;
        public static final int TOP_BEAM_BREAK_PORT = 4;
        public static final int BOTTOM_BEAM_BREAK_PORT = 5;
        public static final int HARD_STOP_SOLENOID_PORT = 5;
        public static final int TOWER_MOTOR_PORT = 23;

        public static final double BLUE_THRESHOLD = 0.3; // Courtesy of Kevin Lobo, if this doesn't work blame him
        public static final double RED_THRESHOLD = 0.3; // Courtesy of Kevin Lobo, if this doesn't work blame him

    }

    public static class FloorIntake {

        public static final int FLOOR_INTAKE_MOTOR_PORT = 22;
        public static final int FLOOR_INTAKE_MOTOR_CURRENT_LIMIT = 0;
        public static final int FORWARD_SOLENOID_PORT = 2;
        public static final int BACKWARD_SOLENOID_PORT = 1;
        public static final double RUN_SPEED = -1.0;
        public static final double REVERSE_SPEED = 1.0;
        public static final double AUTO_REVERSE_SPEED = 0.5;
        public static final int MAX_BALL_COUNT = 2;
        public static final int BALL_LEAVING_COUNT = 40;
        public static final int PNUEMATIC_HUB_PORT = 0;

    }

    public static class Shooter {
        public static final PIDGains SHOOTER_PID_GAINS = new PIDGains(/*100.0 / 2950.0 / 12.0*/ 6e-8, 1e-15, 0.0, 1.0 / 2925.0, 0.0, 0.0, 0.0, 0.0,
                "Shooter");
        // public static final PIDGains SHOOTER_PID_GAINS = new PIDGains(0.087954, 0.0,
        // 0.0, 1.0 / 2800.0, 0.0, 0.0, 0.0, 0.0,
        // "Shooter");
        public static final double DEFAULT_RPM_SET_POINT = 1300.0;
        public static final double DEFAULT_RPM_THRESHOLD = 75.0;
        public static final int SHOOTER_MOTOR_CURRENT_LIMIT = 0;
        public static final int SHOOTER_MOTOR_PORT_BOTTOM = 24;
        public static final int SHOOTER_MOTOR_PORT_TOP = 25;
        public static final int CAN_ENCODER_PORT = 0;
        public static final double DEFAULT_SHOOTER_RPM = 1400.0;
        public static final double RPM_COEFFICIENT = 0.5;

        public static final double MAX_SHOOTER_VELOCITY = 5880.0 / 2.0;
        public static final double MAX_SHOOTER_ACCELERATION = MAX_SHOOTER_VELOCITY;

        public static final double RPM_BOOST = 0;
        public static final double BAD_BALL_RPM = 600.0;
        public static final double MAX_BLUE_RPM = 2950.0;
        public static final double MAX_SILVER_RPM = 2750.0;
        public static final double DEFAULT_HIGH_RPM = 2800.0;
        public static final double DEFAULT_LOW_RPM = 1250.0;
        public static final double MINIMUM_SHOOTING_DISTANCE = 75.0;
        public static final double MAXIMUM_SHOOTING_DISTANCE = 110.0;
        public static final double IDEAL_SHOOTER_DISTANCE = 95.0;
    }

    public static class Drivebase {
        // Max drive base voltage, can be lowered for testing
        public static final double MAX_VOLTAGE = 12.0;

        // TODO empirically measure the drivetrain's maximum velocity or calculate the
        // theoretical.
        // The formula for calculating the theoretical maximum velocity is:
        // <Motor free speed RPM> / 60 * <Drive reduction> * <Wheel diameter meters> *
        // pi
        // By default this value is setup for a Mk3 standard module using Falcon500s to
        // drive.
        // An example of this constant for a Mk4 L2 module with NEOs to drive is:
        // 5880.0 / 60.0 / SdsModuleConfigurations.MK4_L2.getDriveReduction() *
        // SdsModuleConfigurations.MK4_L2.getWheelDiameter() * Math.PI
        public static final double MAX_VELOCITY_METERS_PER_SECOND = 5.0;
        /* 2.0 */ /* 3.0 * 4.96; */// 3.8;
        // 6380.0 / 60.0 *
        // SdsModuleConfigurations.MK3_STANDARD.getDriveReduction() *
        // SdsModuleConfigurations.MK3_STANDARD.getWheelDiameter() * Math.PI;

        public static final double DRIVEBASE_TRACKWIDTH_METERS = Units.inchesToMeters(24.0);
        public static final double DRIVEBASE_WHEELBASE_METERS = Units.inchesToMeters(24.0);

        public static final double MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND = //205.0 * Math.PI;
                /*80.0*/10.0 * MAX_VELOCITY_METERS_PER_SECOND /
                Math.hypot(DRIVEBASE_TRACKWIDTH_METERS / 2.0, DRIVEBASE_WHEELBASE_METERS / 2.0);

        // TODO find empirically at some point
        public static final double MAX_ACCELERATION = 4.0;

        // TODO drive ports
        public static final int FRONT_LEFT_DRIVE_PORT = 10;
        public static final int FRONT_LEFT_TURN_PORT = 14;
        public static final int FRONT_LEFT_TURN_ENCODER_PORT = 6;

        public static final int BACK_LEFT_DRIVE_PORT = 11;
        public static final int BACK_LEFT_TURN_PORT = 15;
        public static final int BACK_LEFT_TURN_ENCODER_PORT = 7;

        public static final int BACK_RIGHT_DRIVE_PORT = 12;
        public static final int BACK_RIGHT_TURN_PORT = 16;
        public static final int BACK_RIGHT_TURN_ENCODER_PORT = 8;

        public static final int FRONT_RIGHT_DRIVE_PORT = 13;
        public static final int FRONT_RIGHT_TURN_PORT = 17;
        public static final int FRONT_RIGHT_TURN_ENCODER_PORT = 9;

        // TODO what is this being used for? Can we just delete it?
        public static final int FRONT_LEFT_TURN_OFFSET = 0;
        public static final int FRONT_RIGHT_TURN_OFFSET = 0;
        public static final int BACK_LEFT_TURN_OFFSET = 0;
        public static final int BACK_RIGHT_TURN_OFFSET = 0;

       

        // public static final PIDGains THETA_CONTROLLER_GAINS = new PIDGains(0.0);
        public static final PIDGains THETA_CONTROLLER_GAINS = new PIDGains(3.0, 0.0, 0.1);//0.125);// 41.543, 0.0, 2.6885);
        
        public static final PIDGains TURN_BODY_PID_GAINS = new PIDGains(0.1, 0.0, 0.0);

        // public static final PIDGains DRIVE_PID_GAINS = new PIDGains(0.0001);
        public static final PIDGains DRIVE_PID_GAINS = new PIDGains(0.0, 0.0, 0.0); //0.0125);//3.0 / 12.0);//0.000625 / 12.0);

        public static final PIDGains TURN_PID_GAINS = new PIDGains(5.0, 0.0, 0.14);

        public static final double MAX_ANGULAR_VELOCITY = 100.0;//40;
        public static final double MAX_ANGULAR_ACCELERATION = 50.0;//20.0;

        public static final double kV = 2.3599/12.0;///* 0.69433 */ 2.5 / 12.0;// 0.7248;
        public static final double kS = 0.71695/12.0;//0.74871 / 12.0;// 0.54214;
        public static final double kA = 0.42457 / 12.0;//0.10884;/// 0.09744;

        public static final double TURN_kS = 0.162;
        public static final double TURN_kV = 0.28;//0.128;

        public static final double MODULE_DRIVE_DISTANCE_CONVERSION_FACTOR = (1.0 * 10 * Math.PI
                * Units.inchesToMeters(3.85)) / (2048.0 * 6.75);

        public static final double MODULE_DRIVE_VELOCITY_CONVERSION_FACTOR = (1.0 * 10 * Math.PI
                * Units.inchesToMeters(3.85)) / (2048.0 * 6.75);

        public static final double DISTANCE_PER_REVOLUTION = 1.0 / 6.75 / 60 * 2.0 * Math.PI * (4.7 / 2.0) * 0.0254; // m/s
        // Module Gearing: 6.75:1 (14->50, 27->17, 15->45)
        // Wheel Diameter: 4"
        // C = 2*pi*r
        // inches to meter: 0.0254

        // BLUE
        public static final double BLUE_FRONT_LEFT_ANGLE_OFFSET = -24.43; // + 180.0;
        public static final double BLUE_FRONT_RIGHT_ANGLE_OFFSET = 121.55; // + 180.0;
        public static final double BLUE_BACK_LEFT_ANGLE_OFFSET = -87.54;// + 180.0;
        public static final double BLUE_BACK_RIGHT_ANGLE_OFFSET = 169.45;// + 180.0;
        
        // public static final double BLUE_FRONT_LEFT_ANGLE_OFFSET = 0.0; // + 180.0;
        // public static final double BLUE_FRONT_RIGHT_ANGLE_OFFSET = 0.0; // + 180.0;
        // public static final double BLUE_BACK_LEFT_ANGLE_OFFSET = 0.0;// + 180.0;
        // public static final double BLUE_BACK_RIGHT_ANGLE_OFFSET = 0.0;// + 180.0;

        // SILVER
        public static final double SILVER_FRONT_LEFT_ANGLE_OFFSET = -161.27; // 0.0;// + 180.0;
        public static final double SILVER_FRONT_RIGHT_ANGLE_OFFSET = -11.33;// 0.0;// + 180.0;
        public static final double SILVER_BACK_LEFT_ANGLE_OFFSET = -33.92;// 0.0;// + 180;
        public static final double SILVER_BACK_RIGHT_ANGLE_OFFSET = -6.4;// 0.0;// + 180;

        // Use to find the new angle offset
        // public static final double FRONT_LEFT_ANGLE_OFFSET = 0;
        // public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;
        // public static final double BACK_LEFT_ANGLE_OFFSET = 0;
        // public static final double BACK_RIGHT_ANGLE_OFFSET = 0;

        // public static final PIDGains AUTO_PID_GAINS = new PIDGains(2.4616);
        public static final PIDGains AUTO_PID_GAINS = new PIDGains(1.0, 0.0, 0.2);//1.6);//0.075);//0.535 DRIVE_PID_GAINS;//new PIDGains(41.543 / 4.0, 0.0, 2.6885);
        // public static final PIDGains AUTO_PID_GAINS = new PIDGains(1.75, 0.0, 0.1);//1.6);//0.075);//0.535 DRIVE_PID_GAINS;//new PIDGains(41.543 / 4.0, 0.0, 2.6885);

        public static final double VISION_ANGLE_TOLERANCE = 3.0;//3.0;

        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 4;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 8;
        public static final int MAGNETIC_SENSOR_PORT = 7;
        public static final double JUKE_DISTANCE = 120.0; // Inches

        public static Constraints kThetaControllerConstraints = new Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);

        public static final double TURN_SLEW_RATE = 5;

        public static final double VISION_KF = ((MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 90.0));// 1.0 / (360.0 /
                                                                                             // MAX_ANGULAR_SPEED_RADIANS_PER_SECOND);

        public static final PIDGains ROTATE_PID_GAINS = new PIDGains(0.1, 0.0, 0.0, VISION_KF);

        public static final PIDGains PREDICTIVE_PID_GAINS = new PIDGains(0.0, 0.0, 0.0,
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND / 45.0);

        public static final double SPEED_REDUCTION_FACTOR = 3.0;
    }

    public static class ControllerInputs {

        public static final double DEADBAND = 0.1;

        public static final int DRIVER_CONTROLLER_PORT = 0;
        public static final int OPERATOR_CONTROLLER_PORT = 1;
        public static final int PROGRAMMER_CONTROLLER_PORT = 2;
    }

    public static class Auto {
        public static final double MAX_AUTO_VELOCITY = 3.0 * Drivebase.MAX_VELOCITY_METERS_PER_SECOND / 4.0; // just max
                                                                                                       // velocity too
                                                                                                       // fast
        
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI * 1.25;
        public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI * 4;         
        public static Constraints kThetaControllerAutoConstraints = new Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND,
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
        public static final double RIGHT_TARMAC_START_ANGLE = -90.0;// 180 - 119.0;
        public static final double FOUR_BALL_NEW_ANGLE_HEADING_AFTER_AUTO = -29.0;
        public static final double ONE_BALL_NEW_ANGLE_HEADING_AFTER_AUTO = 21.0;
        public static final double SHOOT_AND_MOVE_HEADING_AFTER_AUTO = 0.0;
        public static final double SHOOT_AND_MOVE_NEW_ANGLE_HEADING_AFTER_AUTO = 21.0; // 33;
        public static final double LEFT_TARMAC_START_ANGLE = 29.0;
        public static final double TWO_BALL_HIGH_START_ANGLE = -135.0;

    }

    public static class LED {

        public static final int LED_PORT = 10;
        public static final int BOTTOM_START_INDEX = 7;
        public static final int TOP_START_INDEX = 16;
        public static final int FIRING_START_INDEX = 23;
        public static final Color EMPTY_COLOR = Color.kWhite;
        public static final Color IS_SHOOTING_COLOR = Color.kDarkGreen;
        public static final Color ON_TARGET_AND_RPM_COLOR = Color.kLimeGreen;
        public static final Color ON_TARGET_NOT_ON_RPM_COLOR = Color.kLightGoldenrodYellow;
        public static final Color NOT_ON_TARGET_OR_RPM_COLOR = Color.kSlateGray;
        public static final Color NOT_ON_TARGET_ON_RPM_COLOR = Color.kLightGoldenrodYellow;
        public static final int COUNT = 9;

        public static final int LEFT_LED_LENGTH = 24;
        public static final int RIGHT_LED_LENGTH = 26;
        public static final int LED_STRING_LENGTH = LEFT_LED_LENGTH + RIGHT_LED_LENGTH;

    }

    public static class Vision {
        public static final double LENGTH = 2.5;
        
        public static final double LIMELIGHT_HEIGHT = 42.0 / 12.0; // height inches / inches per foot//1.833333;
        public static final double INITIAL_ANGLE = 0.0;
        public static final PIDGains VISION_PID_GAINS = new PIDGains(180.0/Math.PI*(1.0/25.0), 0.0, 0.0);
        public static final double TX_FILTER = 0.75;
        
        public static final PIDGains DRIVE_VISION_PID_GAINS = new PIDGains(5.0 / 50.0);
        public static final Transform2d CAMERA_TO_ROBOT = new Transform2d(new Translation2d(LIMELIGHT_HEIGHT, 0.), new Rotation2d(0.));
        public static final double LIMELIGHT_HEIGHT_METER = Units.feetToMeters(LIMELIGHT_HEIGHT);
        public static final double GOAL_HEIGHT = 65.0 / 12.0; // for april tags, is distance to top of tag
        public static final double GOAL_HEIGHT_METER = Units.feetToMeters(GOAL_HEIGHT);
    }
}