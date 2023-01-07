// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.TalonFXInvertType;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.utilities.DaisyMath;
import frc.robot.utilities.PIDGains;

public class Climber extends SubsystemBase {

  public ClimbSelector climbStage = ClimbSelector.ONE;

  public static boolean climbing = false;
  public static boolean rainbowClimbing = false;

  private static TalonFX mShoulderMasterMotor;
  private TalonFX mShoulderSlaveMotor;

  private static Climber mClimber;

  public static Climber getInstance() {
    if (mClimber != null) {
      return mClimber;
    }
    mClimber = new Climber();
    return mClimber;
  }

  private static CANCoder mShoulderCanCoder;

  public double mOffset;

  public PIDController mShoulderController;
  public SimpleMotorFeedforward mShoulderFeedForward;
  private static double mShoulderAbsoluteOffset = Constants.Climber.CANCODER_ABSOLUTE_POSITION_OFFSET;

  double mSetPoint;

  public boolean mClimbing = false;

  /** Creates a new Climber. */
  public Climber() {

    mShoulderCanCoder = new CANCoder(Constants.Climber.CANCODER_PORT);

    mShoulderMasterMotor = new TalonFX(Constants.Climber.MASTER_MOTOR_PORT);
    mShoulderSlaveMotor = new TalonFX(Constants.Climber.SLAVE_MOTOR_PORT);

    /* Factory Default all hardware to prevent unexpected behaviour */
    mShoulderMasterMotor.configFactoryDefault();
    mShoulderSlaveMotor.configFactoryDefault();

    /* Config the sensor used for Primary PID and sensor direction */
    mShoulderMasterMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Climber.kPIDLoopIdx,
        Constants.Climber.kTimeoutMs);

    mShoulderSlaveMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor,
        Constants.Climber.kPIDLoopIdx,
        Constants.Climber.kTimeoutMs);

    /**
     * Set based on what direction you want forward/positive to be.
     * This does not affect sensor phase.
     */
    mShoulderMasterMotor.setInverted(TalonFXInvertType.Clockwise);
    mShoulderSlaveMotor.setInverted(TalonFXInvertType.FollowMaster);

    /* Ensure sensor is positive when output is positive */
    mShoulderMasterMotor.setSensorPhase(true);
    mShoulderSlaveMotor.setSensorPhase(true);

    /*
     * Talon FX does not need sensor phase set for its integrated sensor
     * This is because it will always be correct if the selected feedback device is
     * integrated sensor (default value)
     * and the user calls getSelectedSensor* to get the sensor's position/velocity.
     * 
     * https://phoenix-documentation.readthedocs.io/en/latest/ch14_MCSensor.html#
     * sensor-phase
     */
    // _talon.setSensorPhase(true);

    /* Config the peak and nominal outputs, 12V means full */
    mShoulderMasterMotor.configNominalOutputForward(0, Constants.Climber.kTimeoutMs);
    mShoulderMasterMotor.configNominalOutputReverse(0, Constants.Climber.kTimeoutMs);
    mShoulderMasterMotor.configPeakOutputForward(1, Constants.Climber.kTimeoutMs);
    mShoulderMasterMotor.configPeakOutputReverse(-1, Constants.Climber.kTimeoutMs);

    mShoulderSlaveMotor.configNominalOutputForward(0, Constants.Climber.kTimeoutMs);
    mShoulderSlaveMotor.configNominalOutputReverse(0, Constants.Climber.kTimeoutMs);
    mShoulderSlaveMotor.configPeakOutputForward(1, Constants.Climber.kTimeoutMs);
    mShoulderSlaveMotor.configPeakOutputReverse(-1, Constants.Climber.kTimeoutMs);

    /**
     * Config the allowable closed-loop error, Closed-Loop output will be
     * neutral within this range. See Table in Section 17.2.1 for native
     * units per rotation.
     */
    mShoulderMasterMotor.configAllowableClosedloopError(0, Constants.Climber.kPIDLoopIdx, Constants.Climber.kTimeoutMs);
    mShoulderSlaveMotor.configAllowableClosedloopError(0, Constants.Climber.kPIDLoopIdx, Constants.Climber.kTimeoutMs);

    mShoulderController = new PIDController(
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kP,
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kI,
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kD);

    mShoulderFeedForward = new SimpleMotorFeedforward(
        Constants.Climber.kS,
        Constants.Climber.kv,
        0.01);

    mShoulderSlaveMotor.follow(mShoulderMasterMotor);

    // Make tuning the shoulder gains available
    Constants.Climber.SHOULDER_MOTOR_PID_GAINS.logToDashboard();
    setShoulderGains();

    mOffset = DaisyMath.minmax(getShoulderPosition(), Constants.Climber.MINIMUM_STARTING_ANGLE,
        Constants.Climber.MAXIMUM_STARTING_ANGLE);

    // Using the cancoder, try to figure out what the offset from the ideal starting
    // position is in order to set the falcon encoders properly
    // so our arm positions will stay the same regardless of where the arm starts on
    // powerup
    double trueSholderPosition = getShoulderAbsolutePosition() * Constants.Climber.CANCODER_TO_OUTPUT_STAGE
        + Constants.Climber.SHOULDER_STARTING_POSITION;
    setShoulderEncoderPosition(trueSholderPosition);
  }

  public void setSpeed(double speed) {
    mShoulderMasterMotor.set(TalonFXControlMode.PercentOutput, speed);
  }

  public void setShoulderGains() {
    // mShoulderMasterMotor.selectProfileSlot(0, 0);
    // mShoulderMasterMotor.config_kP(0,
    // Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kP);
    // mShoulderMasterMotor.config_kI(0,
    // Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kI);
    // mShoulderMasterMotor.config_kP(0,
    // Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kD);
    // mShoulderMasterMotor.config_kF(0,
    // Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kF);

    // mShoulderMasterMotor.configMotionCruiseVelocity(15000, 30);
    // mShoulderMasterMotor.configMotionAcceleration(6000, 30);

    mShoulderController.setPID(
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kP,
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kI,
        Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kD);
  }

  public void setShoulderPosition(double degrees) {
    mSetPoint = degrees;// * Constants.Climber.SHOULDER_TALON_POSITION_FACTOR;
    Double turnOutput = mShoulderController.calculate(mClimber.getShoulderPosition(), degrees);
    Double turnFeedForward = Constants.Climber.SHOULDER_MOTOR_PID_GAINS.kF
        * DaisyMath.boundAngleNeg180to180Degrees(degrees - mClimber.getShoulderPosition());// mShoulderFeedForward.calculate(mShoulderMasterMotor.getSelectedSensorVelocity());

    mShoulderMasterMotor.set(
        TalonFXControlMode.PercentOutput,
        turnOutput + turnFeedForward);

    // SmartDashboard.putNumber("Shoulder/Desired Setpoint", degrees);
    // SmartDashboard.putNumber("Shoulder/Current Angle", mClimber.getShoulderPosition());
    // SmartDashboard.putNumber("Shoulder/Turn Feedforward", turnFeedForward);
    // SmartDashboard.putNumber("Shoulder/Turn Output", turnOutput);

    // mShoulderMasterMotor.set(TalonFXControlMode.Position, mSetPoint);

    // mShoulderMasterMotor.set(TalonFXControlMode.MotionMagic, mSetPoint);
  }

  // Gets the absolute encoder position from the shoulder gearbox output shaft.
  // There is still a 5:1 reduction after,
  // so the arm could be in 5 potential spots based on this number.
  public static double getShoulderAbsolutePosition() {
    return DaisyMath.boundAngleNeg180to180Degrees(mShoulderCanCoder.getAbsolutePosition() - mShoulderAbsoluteOffset);
  }

  public void ResetShoulderPosition(double position) {
    // mShoulderCanCoder.setPosition(position);
    mShoulderController.reset();
  }

  // Degrees
  public double getShoulderPosition() {
    return mShoulderMasterMotor.getSelectedSensorPosition(0) / Constants.Climber.SHOULDER_TALON_POSITION_FACTOR;
  }

  // This is the amount of degrees the shoulder motor encoder needs to be offset
  // by in order to know it's true position
  public static void setShoulderEncoderPosition(double position) {
    mShoulderMasterMotor.setSelectedSensorPosition(position * Constants.Climber.SHOULDER_TALON_POSITION_FACTOR);
  }

  private void logToDashboardInDetail() {

  }
  

  private void logToDashboard() {

    // TODO: Remove these once we have the shoulder initialization code working
    SmartDashboard.putNumber("Climber/CANCoder Position", mShoulderCanCoder.getAbsolutePosition());
    SmartDashboard.putNumber("Climber/TalonFX Position",
        mShoulderMasterMotor.getSelectedSensorPosition() / Constants.Climber.SHOULDER_TALON_POSITION_FACTOR);
    SmartDashboard.putNumber("Climber/TalonFX Raw Position", mShoulderMasterMotor.getSelectedSensorPosition());
    SmartDashboard.putNumber("Climber/Shoulder Position", getShoulderPosition());
    SmartDashboard.putNumber("Climber/Absolute Shoulder Position", getShoulderAbsolutePosition());
    SmartDashboard.putNumber("Climber/Raw Absolute Shoulder Position",
        DaisyMath.boundAngleNeg180to180Degrees(getShoulderAbsolutePosition() + mShoulderAbsoluteOffset));
    SmartDashboard.putNumber("Climber/Shoulder Set Point", mSetPoint);
    SmartDashboard.putString("Climber/Stage", climbStage.toString());

  }

  @Override
  public void periodic() {
    if (Robot.detailedLogging) {
      logToDashboardInDetail();
    }
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("Climber");
    // This method will be called once per scheduler run
  }
}
