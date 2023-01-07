// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.led.ColorFlowAnimation.Direction;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMax.SoftLimitDirection;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.PIDGains;

public class Hook extends SubsystemBase {
  private String mName;

  private Double mGoalHookPosition = 0.0;

  private DigitalInput mLimitSwitch;

  private CANSparkMax mHookMotor;

  private Double mAlphaPosition = 0.0;
  private double mReverseSoftLimit;
  private double mForwardSoftLimit;

  private boolean mInvertLimitSwitch;


  private void HookSetup(String name, int motorPort, int limitSwitchPort, boolean isInverted, float reverseSoftLimit, float forwardSoftLimit, boolean invertLimitSwitch) {
    
    mReverseSoftLimit = (double) reverseSoftLimit;
    mForwardSoftLimit = (double) forwardSoftLimit;
    mHookMotor = new CANSparkMax(motorPort, MotorType.kBrushless);
    mHookMotor.setIdleMode(IdleMode.kBrake);
    mHookMotor.getPIDController().setP(Constants.Climber.HOOK_MOTOR_PID_GAINS.kP);
    mHookMotor.getEncoder().setPositionConversionFactor(Constants.Climber.HOOK_POSITION_CONVERSION_FACTOR);

    mHookMotor.setInverted(isInverted);

    mLimitSwitch = new DigitalInput(limitSwitchPort);

    mName = name;

    int smartMotionSlot = 0;
    mHookMotor.getPIDController().setSmartMotionMaxVelocity(2000.0, smartMotionSlot);
    mHookMotor.getPIDController().setSmartMotionMinOutputVelocity(0.0, smartMotionSlot);
    mHookMotor.getPIDController().setSmartMotionMaxAccel(1500.0, smartMotionSlot);
    mHookMotor.getPIDController().setSmartMotionAllowedClosedLoopError(5.0, smartMotionSlot);
    mHookMotor.getPIDController().setOutputRange(-1.0, 1.0);

    mHookMotor.setSoftLimit(SoftLimitDirection.kForward, (float) mForwardSoftLimit); /// Constants.Climber.HOOK_POSITION_CONVERSION_FACTOR));
    mHookMotor.setSoftLimit(SoftLimitDirection.kReverse, (float) mReverseSoftLimit);

    mHookMotor.enableSoftLimit(SoftLimitDirection.kForward, true);
    mHookMotor.enableSoftLimit(SoftLimitDirection.kReverse, true);

    mInvertLimitSwitch = invertLimitSwitch;

  }

  /** Creates a new Hook. */
  public Hook(String name, int motorPort, int limitSwitchPort, boolean isInverted, float reverseSoftLimit, float forwardSoftLimit) {

    HookSetup(name,motorPort,limitSwitchPort,isInverted,reverseSoftLimit,forwardSoftLimit,false);
  }
  public Hook(String name, int motorPort, int limitSwitchPort, boolean isInverted, float reverseSoftLimit, float forwardSoftLimit, boolean invertLimitSwitch) {
    HookSetup(name,motorPort,limitSwitchPort,isInverted,reverseSoftLimit,forwardSoftLimit,invertLimitSwitch);
  }

  /**
   * Sets hook motor PIDs
   */
  public void setPIDs(PIDGains pid) {
    mHookMotor.getPIDController().setP(Constants.Climber.HOOK_MOTOR_PID_GAINS.kP);
    mHookMotor.getPIDController().setI(Constants.Climber.HOOK_MOTOR_PID_GAINS.kI);
    mHookMotor.getPIDController().setD(Constants.Climber.HOOK_MOTOR_PID_GAINS.kD);
    mHookMotor.getPIDController().setFF(Constants.Climber.HOOK_MOTOR_PID_GAINS.kF);
  }

  /**
   * @return limit switch state
   */
  public boolean getLimitSwitchState() {
    if (mInvertLimitSwitch) return mLimitSwitch.get();
    else return !mLimitSwitch.get();
  }

  /**
   * @return hook motor PID controller
   */
  public SparkMaxPIDController getPID() {
    return mHookMotor.getPIDController();
  }

  /**
   * Sets hook position
   * 
   * @param degrees desired angle
   * @param alpha
   */
  public void setPosition(double degrees, double alpha) {
    mGoalHookPosition = degrees;
    mAlphaPosition = alpha * degrees + (1 - alpha) * mAlphaPosition;
    getPID().setReference(mAlphaPosition, CANSparkMax.ControlType.kPosition);
  }

  /**
   * @return current hook position
   */
  public double getDistance() {
    return mHookMotor.getEncoder().getPosition();
  }

  public double getReverseSoftLimit() {
    return mReverseSoftLimit;
  }

  public double getForwardSoftLimit() {
    return mForwardSoftLimit;
  }

  /**
   * @return current hook position
   */
  public double getPosition() {
    return mHookMotor.getEncoder().getPosition();
  }

  /**
   * Updates hook motor PID values
   * 
   * @param PIDs
   */
  public void setPID(PIDGains PIDs) {
    getPID().setP(PIDs.kP);
    getPID().setI(PIDs.kI);
    getPID().setD(PIDs.kD);
    getPID().setFF(PIDs.kF);
    getPID().setOutputRange(PIDs.kMinOutput, PIDs.kMaxOutput);
  }

  /**
   * Sets hook speed
   * 
   * @param speed bound by -1.0 to 1.0
   */
  public void setSpeed(double speed) {
    mHookMotor.set(speed);
  }

  /**
   * @return whether the hook is in its desired position
   */
  public boolean hookInPosition() {
    return Math.abs(mGoalHookPosition - getPosition()) <= Constants.Climber.HOOK_ANGLE_TOLERANCE;
  }

  public void resetPosition(double position) {
    mHookMotor.getEncoder().setPosition(position);
  }

  private void logToDashboard() {
    // SmartDashboard.putBoolean("Hook/" + mName +"/ Limit Switch", getLimitSwitchState());
    SmartDashboard.putNumber("Hook/" + mName + "/ position", getPosition());
  }

  // private void logToDashdoardInDetail() {
  //   SmartDashboard.putNumber("Hook/" + mName + "/ goal position", mGoalHookPosition);
  //   SmartDashboard.putBoolean("Hook/" + mName + "/ in position", hookInPosition());
  //   SmartDashboard.putNumber("Hook/" + mName + "/ raw position", mHookMotor.getEncoder().getPosition());
  // }

  @Override
  public void periodic() {
    // if (Robot.detailedLogging) {
    //   logToDashdoardInDetail();
    // }
    // This method will be called once per scheduler run
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("Hook");
  }

}