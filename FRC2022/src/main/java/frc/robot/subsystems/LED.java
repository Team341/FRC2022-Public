// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import javax.print.attribute.IntegerSyntax;

import com.ctre.phoenix.ErrorCode;
import com.ctre.phoenix.led.CANdle;
import com.ctre.phoenix.led.CANdleConfiguration;
import com.ctre.phoenix.led.RainbowAnimation;
import com.ctre.phoenix.led.CANdle.LEDStripType;
import com.ctre.phoenix.led.CANdle.VBatOutputMode;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Tower.BallState;
import frc.robot.subsystems.Shooter.firingState;

import frc.robot.subsystems.tracking.LimelightInterface;

public class LED extends SubsystemBase {

  private static LED mInstance;

  public static LED getInstance() {
    if (mInstance == null) {
      mInstance = new LED();
    }
    return mInstance;
  }

  private final CANdle mLED;
  private boolean isAuto;
  private BallState mBottomBallState;
  private BallState mTopBallState;
  private Shooter.firingState mFiringState;

  private CANdleConfiguration config;

  private Tower mTower = Tower.getInstance();
  private LimelightInterface mPhotonVisionInterface = LimelightInterface.getInstance();
  private Shooter mShooter = Shooter.getInstance();

  /** Creates a new LED. */
  public LED() {
    isAuto = false;
    mLED = new CANdle(Constants.LED.LED_PORT);
    config = new CANdleConfiguration();
    config.statusLedOffWhenActive = false;
    config.disableWhenLOS = false;
    config.stripType = LEDStripType.RGB;
    config.brightnessScalar = 0.5;
    config.vBatOutputMode = VBatOutputMode.On;

    mLED.configAllSettings(config, 100);
    // mLED.configFactoryDefault();

    mBottomBallState = BallState.Empty;
    mTopBallState = BallState.Empty;
    mFiringState = firingState.NotOnTargetOrRPM;
  }

  public void setAutoLED() {
    isAuto = true;
  }

  public void setTeleOpLED() {
    isAuto = false;
  }

  public boolean isUsingAutoLED() {
    return isAuto;
  }

  /**
   * Sets bottom color
   * 
   * @param color desired RGB values
   */
  public void setBottomLEDColor(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.BOTTOM_START_INDEX, // 7
        Constants.LED.LEFT_LED_LENGTH / 3); // lights 8

    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.BOTTOM_START_INDEX + Constants.LED.LEFT_LED_LENGTH, // 31
        Constants.LED.RIGHT_LED_LENGTH / 3); // lights 9
  }

  /**
   * Sets top color
   * 
   * @param color desired RGB values
   */
  public void setTopLEDColor(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.TOP_START_INDEX, // 16
        Constants.LED.LEFT_LED_LENGTH / 3); // lights 8

    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.TOP_START_INDEX + Constants.LED.LEFT_LED_LENGTH, // 40
        Constants.LED.RIGHT_LED_LENGTH / 3); // lights 9
  }

  /**
   * Sets color when shooting
   * 
   * @param color desired RGB values
   */
  public void setFiringLEDColor(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.FIRING_START_INDEX, // 25
        Constants.LED.LEFT_LED_LENGTH / 3); // lights 8

    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.FIRING_START_INDEX + Constants.LED.LEFT_LED_LENGTH, // 47
        Constants.LED.RIGHT_LED_LENGTH / 3); // lights 9 [OUT OF BOUNDS]
  }

  public void setLeftShortLEDs(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.BOTTOM_START_INDEX,
        Constants.LED.LEFT_LED_LENGTH / 2);
  }

  public void setLeftLongLEDs(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.BOTTOM_START_INDEX + Constants.LED.LEFT_LED_LENGTH / 2 /* + Constants.LED.RIGHT_LED_LENGTH % 4 */,
        Constants.LED.LEFT_LED_LENGTH / 2);
  }

  public void setRightShortLEDs(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.BOTTOM_START_INDEX + Constants.LED.LEFT_LED_LENGTH,
        (Constants.LED.RIGHT_LED_LENGTH) / 2);
  }

  public void setRightLongLEDs(Color color) {
    mLED.setLEDs(
        (int) color.red * 255,
        (int) color.green * 255,
        (int) color.blue * 255,
        120,
        Constants.LED.LEFT_LED_LENGTH + ((Constants.LED.RIGHT_LED_LENGTH) / 2),
        (Constants.LED.RIGHT_LED_LENGTH) / 2);
  }

  // Wrapper functions

  /**
   * @return bus voltage
   */
  public double getBatteryVoltage() {
    return mLED.getBusVoltage();
  }

  /**
   * @return rail voltage
   */
  public double get5V() {
    return mLED.get5VRailVoltage();
  }

  /**
   * @return low side current
   */
  public double getCurrent() {
    return mLED.getCurrent();
  }

  /**
   * @return temperature in celcius
   */
  public double getTemperature() {
    return mLED.getTemperature();
  }

  /**
   * @param percent scaling brightness
   */
  public void configBrightness(double percent) {
    mLED.configBrightnessScalar(percent, 0);
  }

  /**
   * @param disableWhenLos whether to disable LED when signal is lost
   */
  public void configLos(boolean disableWhenLos) {
    mLED.configLOSBehavior(disableWhenLos, 0);
  }

  /**
   * @param type the type of LED
   */
  public void configLedType(LEDStripType type) {
    mLED.configLEDType(type, 0);
  }

  /**
   * @param offWhenActive whether LED is off when CANdle is activated
   */
  public void configStatusLedBehavior(boolean offWhenActive) {
    mLED.configStatusLedState(offWhenActive, 0);
  }

  private void logToDashboard() {

  }

  // private void logToDashboardInDetail() {
  //   SmartDashboard.putNumber("LED/Current", getCurrent());
  //   SmartDashboard.putNumber("LED/Temperature", getTemperature());
  //   SmartDashboard.putNumber("LED/Battery Voltage", getBatteryVoltage());
  //   SmartDashboard.putNumber("LED/5V Rail Voltage", get5V());

  // }

  /**
   * @return LED
   */
  public CANdle getLED() {
    return mLED;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // if (Robot.detailedLogging) {
    //   logToDashboardInDetail();
    // }
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("LED");
    // switch (mTower.getBottomBallState()) {
    // case Empty:
    // mBottomBallState = BallState.Empty;
    // setBottomLEDColor(Constants.LED.EMPTY_COLOR);
    // break;
    // case Red:
    // mBottomBallState = BallState.Red;
    // setBottomLEDColor(Color.kRed);
    // break;
    // case Blue:
    // mBottomBallState = BallState.Blue;
    // setBottomLEDColor(Color.kBlue);
    // }

    // switch (mTower.getBottomBallState()) {
    // case Empty:
    // mTopBallState = BallState.Empty;
    // setTopLEDColor(Constants.LED.EMPTY_COLOR);
    // break;
    // case Red:
    // mTopBallState = BallState.Red;
    // setTopLEDColor(Color.kRed);
    // break;
    // case Blue:
    // mTopBallState = BallState.Blue;
    // setTopLEDColor(Color.kBlue);
    // }

    // if (mShooter.isShooting()) {
    // mFiringState = firingState.Shooting;
    // setFiringLEDCOlor(Constants.LED.IS_SHOOTING_COLOR);
    // } else if (mLimelightInterface.alignedToGoal()) {
    // if (mShooter.getOnRPMTarget()) {
    // mFiringState = firingState.OnTargetAndRPM;
    // setFiringLEDCOlor(Constants.LED.ON_TARGET_AND_RPM_COLOR);
    // } else {
    // mFiringState = firingState.OnTargetNotOnRPM;
    // setFiringLEDCOlor(Constants.LED.ON_TARGET_NOT_ON_RPM_COLOR);
    // }
    // } else {
    // if (mShooter.getOnRPMTarget()) {
    // mFiringState = firingState.NotOnTargetOnRPM;
    // setFiringLEDCOlor(Constants.LED.NOT_ON_TARGET_ON_RPM_COLOR);
    // } else {
    // mFiringState = firingState.NotOnTargetOrRPM;
    // setFiringLEDCOlor(Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR);
    // }
    // }
  }
}