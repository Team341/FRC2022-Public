// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class FloorIntake extends SubsystemBase {
  /** Creates a new FloorIntake. */
  private static FloorIntake instance = null;

  public static FloorIntake getInstance() {
    if (instance == null) {
      instance = new FloorIntake();
    }
    return instance;
  }

  // creates a new motor object for intake wheels
  private final CANSparkMax mFloorIntakeMotor;
  // private final CANSparkMax mFloorIntakeMotorTwo;

  // creates a new solenoid object
  private final DoubleSolenoid mSolenoid;

  private boolean mIntakeState; // true if deployed

  public void deployIntake() {

    if (!mIntakeState) {
      mSolenoid.set(Value.kForward);
      mIntakeState = true;
    }

  }

  // PneumaticHub pneumaticHub;

  public FloorIntake() {
    // Setup the floor intake motor motor (1x Neo550)
    mFloorIntakeMotor = new CANSparkMax(Constants.FloorIntake.FLOOR_INTAKE_MOTOR_PORT, MotorType.kBrushless);

    // Configure intake motor current and Idle mode
    mFloorIntakeMotor.restoreFactoryDefaults();
    mFloorIntakeMotor.setSmartCurrentLimit(Constants.FloorIntake.FLOOR_INTAKE_MOTOR_CURRENT_LIMIT);
    mFloorIntakeMotor.setIdleMode(IdleMode.kCoast);
    mFloorIntakeMotor.setInverted(true);

    // mFloorIntakeMotor.getEncoder(EncoderType.kNoSensor, 0);

    // Burn configuration to flash memory to prevent issues if power is lost while
    // robot is operating
    mFloorIntakeMotor.burnFlash();

    // instantiates solenoid object to new Solenoid class
    mSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.FloorIntake.BACKWARD_SOLENOID_PORT,
        Constants.FloorIntake.FORWARD_SOLENOID_PORT);

    mIntakeState = false;

    // pneumaticHub = new PneumaticHub();
    setFloorIntakeState(false);
  }

  /**
   * gets the state of the floor intake if it's lowered or not
   * 
   * @return boolean true = down, false = up
   */
  public Boolean getFloorIntakeState() {
    mIntakeState = mSolenoid.get().equals(Value.kForward);
    return mIntakeState;
  }

  /**
   * gets the voltage double of the motor
   * 
   * @return returns the motor's voltage
   */
  public double getMotorCommandVoltage() {
    return mFloorIntakeMotor.getBusVoltage();
  }

  /**
   * runs the intake at the reverse speed defined in Constants
   */
  public void reverseIntake() {
    setFloorIntakeState(true);
    mFloorIntakeMotor.set(Constants.FloorIntake.REVERSE_SPEED);
  }

  /**
   * runs the intake at the reverse speed defined in Constants
   */
  public void autoReverseIntake() {
    setFloorIntakeState(true);
    mFloorIntakeMotor.set(Constants.FloorIntake.AUTO_REVERSE_SPEED);
  }

  /**
   * runs the intake at the speed defined in Constants
   */
  public void runIntake() {

    setFloorIntakeState(true);
    mFloorIntakeMotor.set(Constants.FloorIntake.RUN_SPEED);
  }

  /**
   * sets the boolean state of the floor intake
   * 
   * @param State
   */
  public void setFloorIntakeState(boolean state) {
    mIntakeState = state;

    if (state) {
      mSolenoid.set(Value.kForward);
    } else {
      mSolenoid.set(Value.kReverse);
    }

  }

  /**
   * sets the speed double of the intake
   * 
   * @param speed
   */
  public void setIntakeSpeed(double speed) {
    mFloorIntakeMotor.set(speed);
  }

  /**
   * retracts intake if deployed
   */
  public void retractIntake() {

    if (mIntakeState) {
      mSolenoid.set(Value.kReverse);
      mIntakeState = false;
    }

  }

  // private void logToDashboardInDetail() {
  //   SmartDashboard.putNumber("Floor Intake/Motor Command Voltage", mFloorIntakeMotor.getBusVoltage());
  // }

  private void logToDashboard() {
    // SmartDashboard.putBoolean("Floor Intake/Is Floor Intake Lowered", mIntakeState);
    // SmartDashboard.putBoolean("Floor Intake/Is Motor Going Backwards", getMotorGoingBackwards());

  }

  @Override
  public void periodic() {
    // if (Robot.detailedLogging) {
    //   logToDashboardInDetail();
    // }
    // This method will be called once per scheduler run
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("FloorIntake");
  }

  public boolean getMotorGoingBackwards() {
    return mFloorIntakeMotor.get() > 0.0;
  }

  public boolean getMotorGoingForwards() {
    return mFloorIntakeMotor.get() < 0.0;
  }
}