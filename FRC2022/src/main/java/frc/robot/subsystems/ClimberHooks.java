// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.PIDGains;

public class ClimberHooks extends SubsystemBase {
  /** Creates a new ClimberHooks. */


  private static ClimberHooks mClimberHooks;
  public static ClimberHooks getInstance() {
    if (mClimberHooks != null) {
      return mClimberHooks;
    }
    mClimberHooks = new ClimberHooks();
    return mClimberHooks;
  }


  private Hook mHookLeftLongSide;
  private Hook mHookLeftShortSide;
  private Hook mHookRightShortSide;
  private Hook mHookRightLongSide;
  private double mHookAlpha = Constants.Climber.DEFAULT_HOOK_ALPHA;

  public ClimberHooks() {


    mHookLeftLongSide = new Hook("Left Long", Constants.Climber.LEFT_LONG_HOOK_MOTOR_PORT, Constants.Climber.LEFT_LONG_LIMIT_SWITCH, true, (float) Constants.Climber.LEFT_LONG_CLOSED_POSITION, (float) Constants.Climber.LEFT_LONG_OPEN_POSITION);
    mHookLeftShortSide = new Hook("Left Short", Constants.Climber.LEFT_SHORT_HOOK_MOTOR_PORT, Constants.Climber.LEFT_SHORT_LIMIT_SWITCH, true, (float) Constants.Climber.LEFT_SHORT_CLOSED_POSITION, (float) Constants.Climber.LEFT_SHORT_OPEN_POSITION);
    mHookRightShortSide = new Hook("Right Short", Constants.Climber.RIGHT_SHORT_HOOK_MOTOR_PORT, Constants.Climber.RIGHT_SHORT_LIMIT_SWITCH, false, (float) Constants.Climber.RIGHT_SHORT_CLOSED_POSITION, (float) Constants.Climber.RIGHT_SHORT_OPEN_POSITION, true);
    mHookRightLongSide = new Hook("Right Long", Constants.Climber.RIGHT_LONG_HOOK_MOTOR_PORT, Constants.Climber.RIGHT_LONG_LIMIT_SWITCH, false, (float) Constants.Climber.RIGHT_LONG_CLOSED_POSITION, (float) Constants.Climber.RIGHT_LONG_OPEN_POSITION);

    // Make tuning the hook gains available
    Constants.Climber.HOOK_MOTOR_PID_GAINS.logToDashboard();
    SmartDashboard.putNumber("Hook/Alpha", mHookAlpha);
  }

  /**
   * Sets motor speed of left long side
   * @param speed bound between -1.0 and 1.0
   */
  public void setHookMotorLeftLongSideSpeed(double speed) {
    mHookLeftLongSide.setSpeed(speed);
  }
  /**
   * Sets motor speed of left short side
   * @param speed bound between -1.0 and 1.0
   */
  public void setHookMotorLeftShortSideSpeed(double speed) {
    mHookLeftShortSide.setSpeed(speed);
  }
  /**
   * Sets motor speed of right short side
   * @param speed bound between -1.0 and 1.0
   */
  public void setHookMotorRightShortSideSpeed(double speed) {
    mHookRightShortSide.setSpeed(speed);
  }
  /**
   * Sets motor speed of right long side
   * @param speed bound between -1.0 and 1.0
   */
  public void setHookMotorRightLongSideSpeed(double speed) {
    mHookRightLongSide.setSpeed(speed);
  }

  public void resetAllHookPosition(double position) {
    mHookLeftLongSide.resetPosition(position);
    mHookRightLongSide.resetPosition(position);
    mHookLeftShortSide.resetPosition(position);
    mHookRightShortSide.resetPosition(position);
  }

  /**
   * Sets all hook speeds
   * @param speed bound between -1.0 and 1.0
   */
  public void setAllHookSpeed(double speed) {
    setHookMotorLeftLongSideSpeed(speed);
    setHookMotorLeftShortSideSpeed(speed);
    setHookMotorRightLongSideSpeed(speed);
    setHookMotorRightShortSideSpeed(speed);
  }

  /**
   * Sets all hook PID values
   * @param pid
   */
  public void setHookPIDs(PIDGains pid) {
    mHookLeftLongSide.setPID(pid);
    mHookLeftShortSide.setPID(pid);
    mHookRightLongSide.setPID(pid);
    mHookRightShortSide.setPID(pid);
  }
  /**
   * Sets long side position
   * @param degrees
   * @param alpha
   */
  public void setLongSidePosition(double degrees, double alpha) {
    mHookRightLongSide.setPosition(degrees, alpha);
    mHookLeftLongSide.setPosition(degrees, alpha);
  }
  /**
   * Sets short side position
   * @param degrees
   * @param alpha
   */
  public void setShortSidePosition(double degrees, double alpha) {
    mHookRightShortSide.setPosition(degrees, alpha);
    mHookLeftShortSide.setPosition(degrees, alpha);
  }

  public void setLongSidePositionClosed(double alpha) {
    mHookLeftLongSide.setPosition(mHookLeftLongSide.getForwardSoftLimit(), alpha);
    mHookRightLongSide.setPosition(mHookRightLongSide.getForwardSoftLimit(), alpha);
  }

  public void setShortSidePositionClosed(double alpha) {
    mHookLeftShortSide.setPosition(mHookLeftShortSide.getForwardSoftLimit(), alpha);
    mHookRightShortSide.setPosition(mHookRightShortSide.getForwardSoftLimit(), alpha);
  }

  /**
   * @return left long side limit switch state
   */
  public boolean getLeftLongLimitSwitch() {
    return mHookLeftLongSide.getLimitSwitchState();
  }
  /**
   * @return left short side limit switch state
   */
  public boolean getLeftShortLimitSwitch() {
    return mHookLeftShortSide.getLimitSwitchState();
  }
  /**
   * @return right long side limit switch state
   */
  public boolean getRightLongLimitSwitch() {
    return mHookRightLongSide.getLimitSwitchState();
  }
  /**
   * @return rigth short side limit switch state
   */
  public boolean getRightShortLimitSwitch() {
    return mHookRightShortSide.getLimitSwitchState();
  }

  public boolean getShortSideEngaged(){
    return getRightShortLimitSwitch() && getLeftShortLimitSwitch();
  }

  public boolean getLongSideEngaged(){
    return getRightLongLimitSwitch() && getLeftLongLimitSwitch();
  }
  
  public double getHookAlpha() {
    return mHookAlpha;
  }

  /**
   * @return whether long side hook in position
   */
  public boolean longSideHooksInPosition() {
    return mHookLeftLongSide.hookInPosition() && mHookRightLongSide.hookInPosition(); 
  }
  /**
   * @return whether short side hook in position
   */
  public boolean shortSideHooksInPosition() {
    return mHookLeftShortSide.hookInPosition() && mHookRightShortSide.hookInPosition();
  }
  
  // private void logToDashboardInDetail() {
  //   SmartDashboard.putBoolean("Climber/Short side limit switches", getShortSideEngaged());
  //   SmartDashboard.putBoolean("Climber/Long side limit switches", getLongSideEngaged());
  //   SmartDashboard.putBoolean("Climber/Short side in position", shortSideHooksInPosition());
  //   SmartDashboard.putBoolean("Climber/Long side in position", longSideHooksInPosition());
  //   mHookAlpha = SmartDashboard.getNumber("Hook/Alpha", mHookAlpha);
  // }
  private void logToDashboard() {
    SmartDashboard.putBoolean("Climber/Left Long Limit Switch", mHookLeftLongSide.getLimitSwitchState());
    SmartDashboard.putBoolean("Climber/Left Short Limit Switch", mHookLeftShortSide.getLimitSwitchState());
    SmartDashboard.putBoolean("Climber/Right Long Limit Switch", mHookRightLongSide.getLimitSwitchState());
    SmartDashboard.putBoolean("Climber/Right Short Limit Switch", mHookRightShortSide.getLimitSwitchState());
    
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    if (Robot.detailedLogging) {
      // logToDashboardInDetail();
    }
    if (Robot.logging) {
      logToDashboard();
    }
    else {
      SmartDashboard.putBoolean("Climber/Left Long Limit Switch", mHookLeftLongSide.getLimitSwitchState());
      SmartDashboard.putBoolean("Climber/Left Short Limit Switch", mHookLeftShortSide.getLimitSwitchState());
      SmartDashboard.putBoolean("Climber/Right Long Limit Switch", mHookRightLongSide.getLimitSwitchState());
      SmartDashboard.putBoolean("Climber/Right Short Limit Switch", mHookRightShortSide.getLimitSwitchState());
    }
    
    // System.out.println("ClimberHooks");
  }
  
}
