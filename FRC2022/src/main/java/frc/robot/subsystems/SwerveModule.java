// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.utilities.DaisyMath;

public class SwerveModule extends SubsystemBase {

  private TalonFX mDriveMotor;
  private CANSparkMax mTurningMotor;
  
  private CANCoder mTurningEncoder;
  private double mAngleOffset;

  private String name = "Module";

  // Drive Gains
  private final PIDController mDrivePIDController = new PIDController(Constants.Drivebase.DRIVE_PID_GAINS.kP,
      Constants.Drivebase.DRIVE_PID_GAINS.kI,
      Constants.Drivebase.DRIVE_PID_GAINS.kD);

  // Turn Gains
  private final ProfiledPIDController mTurningPIDController = new ProfiledPIDController(
      Constants.Drivebase.TURN_PID_GAINS.kP,
      Constants.Drivebase.TURN_PID_GAINS.kI,
      Constants.Drivebase.TURN_PID_GAINS.kD,
      new TrapezoidProfile.Constraints(
          Constants.Drivebase.MAX_ANGULAR_VELOCITY, Constants.Drivebase.MAX_ANGULAR_ACCELERATION));

  // TODO TODO TODO The swerve modules need to be characterized as a general mechanism, while the drivebase should be characterized as a drivebase
  // TODO these must be seperate variables!!!
  private final SimpleMotorFeedforward mDriveFeedforward = new SimpleMotorFeedforward(Constants.Drivebase.kS,
      Constants.Drivebase.kV);
  private final SimpleMotorFeedforward mTurnFeedforward = new SimpleMotorFeedforward(Constants.Drivebase.TURN_kS,
      Constants.Drivebase.TURN_kV);
  // 0.162, 0.128);

  /**
   * Creates a new SwerveModule.
   * 
   * @param drivePort:       The port number of the drive motor
   * @param turnPort:        The port number of the turn motor
   * @param turnEncoderPort: The port number of the turn encoder
   * @param angleOffset:     The angle offset of the swerve module
   */
  public SwerveModule(int drivePort, int turnPort, int turnEncoderPort, double angleOffset) {
    mDriveMotor = new TalonFX(drivePort);
    mTurningMotor = new CANSparkMax(turnPort, MotorType.kBrushless);
    mTurningEncoder = new CANCoder(turnEncoderPort);
    this.mAngleOffset = angleOffset;

    mDriveMotor.configFactoryDefault();
    mTurningMotor.restoreFactoryDefaults();
    mDriveMotor.setInverted(false);
    mTurningMotor.setInverted(true);

    mDriveMotor.setNeutralMode(NeutralMode.Brake);
    mTurningMotor.setIdleMode(IdleMode.kBrake);

    mDriveMotor.configOpenloopRamp(0.5);


    // Burn the speed controller settings to flash
    mTurningMotor.burnFlash();

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    mTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);
    SmartDashboard.putNumber("Auto/auto_kp", Constants.Drivebase.AUTO_PID_GAINS.kP);

  }

  /**
  * @param name of swerve module
  */
  public void setName(String name) {
    this.name = name;
  }

  /**
   * @param angleOffset in degrees
   */
  public void setAngleOffset(double angleOffset) {
    mAngleOffset = angleOffset;
  }

  /**
   * @return angleOffset in degrees
   */
  public double getAngleOffset() {
    return mAngleOffset;
  }

  /**
  * Read in and convert the analog turn encoder from a voltage to an angle (-180,
  * 180) [0, 360)
  */
  protected double readAngle() {
    // double angle = (1.0 - mTurningEncoder.getVoltage() / RobotController.getVoltage5V()) * 2.0 * Math.PI - mAngleOffset;
    // angle %= 2.0 * Math.PI;
    // if (angle < 0.0) {
    //   angle += 2.0 * Math.PI;
    // }
    // return angle;
    return DaisyMath.boundAngleNeg180to180Degrees(mTurningEncoder.getAbsolutePosition() - mAngleOffset);
  }

  /**
   * Drive PID gains
   * @param kp
   * @param ki
   * @param kd
   */
  public void setDrivePidGains(double kp, double ki, double kd) {
    mDrivePIDController.setPID(kp, ki, kd);
  }

  /**
   *  Turn PID Gains
   * @param kp
   * @param ki
   * @param kd
  */
  public void setTurnPidGains(double kp, double ki, double kd) {
    mTurningPIDController.setPID(kp, ki, kd);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getSpeed(), Rotation2d.fromDegrees(readAngle()));
  }

  /**
   * Get drive module speed
   * @return Speed in meters
   */
  public double getSpeed() {
    return mDriveMotor.getSelectedSensorVelocity() * Constants.Drivebase.MODULE_DRIVE_VELOCITY_CONVERSION_FACTOR;
  }

  /**
   * Get drive module distance traveled
   * @return distance in meters
   */
  public double getDistance() {
    return mDriveMotor.getSelectedSensorPosition() * Constants.Drivebase.MODULE_DRIVE_DISTANCE_CONVERSION_FACTOR / 10.0;
  }

  /**
   * Resets drive encoder
  */
  public void resetDriveEncoder() {
    mDriveMotor.setSelectedSensorPosition(0.0);
  }

  public void setIdleMode(NeutralMode mode) {
    mDriveMotor.setNeutralMode(mode);
  }

  /**
   * Sets the desired state for the module.
   *
   * @param desiredState Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState desiredState) {
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state = SwerveModuleState.optimize(desiredState, new Rotation2d(Math.toRadians(readAngle())));

    // Calculate the drive output from the drive PID controller.
    double driveOutput = mDrivePIDController.calculate(getSpeed(),
        state.speedMetersPerSecond);

    final double driveFeedforward = mDriveFeedforward.calculate(state.speedMetersPerSecond);

    // Calculate the turning motor output from the turning PID controller.
    double turnOutput = mTurningPIDController.calculate(Math.toRadians(readAngle()), state.angle.getRadians());

    final double turnFeedforward = mTurnFeedforward.calculate(mTurningPIDController.getSetpoint().velocity);
    // double turnError = DaisyMath.boundAngleNeg180to180Degrees(readAngle() - state.angle.getDegrees());
    // final double turnFeedforward = mTurnFeedforward.calculate(turnError);

    mDriveMotor.set(TalonFXControlMode.PercentOutput, (driveOutput + driveFeedforward));
    mTurningMotor.setVoltage(turnOutput + turnFeedforward);

    // SmartDashboard.putNumber("Drive/Module: " + name + "Turn Feedforward", turnFeedforward);
    // SmartDashboard.putNumber("Drive/Module: " + name + "/Desired State Angle", state.angle.getDegrees());
    // SmartDashboard.putNumber("Drive/Module: " + name + "/Desired State Angle", state.angle.getDegrees());
    // SmartDashboard.putNumber("Testing/" + name + "/Desired State Speed", state.speedMetersPerSecond);
    // SmartDashboard.putNumber("Testing/"+ name + "/ Drive feedforward", driveFeedforward);
    // SmartDashboard.putNumber("Testing/"+ name + "/ Drive PID output", driveOutput);
    // SmartDashboard.putNumber("Drive/Module: " + name + "Turn Output", turnOutput);
    // SmartDashboard.putNumber("Drive/Module: " + name + "Read angle", readAngle());
    // SmartDashboard.putNumber("Drive/Module: " + name + "Turning PID Controllers", mTurningPIDController.getSetpoint().velocity);
    // SmartDashboard.putNumber("Drive/Module: " + name + " Turn Error", turnError);
  }

  public void setTurnMotorInverted(boolean isInverted) {
    mTurningMotor.setInverted(isInverted);
    mTurningMotor.burnFlash();
  }

  public void setFalconPercentOutput(double output) {
    mDriveMotor.set(TalonFXControlMode.PercentOutput, output);
  }

  public void setBrakeMode(boolean breakMode) {
    mTurningMotor.setIdleMode(IdleMode.kBrake);
    mTurningMotor.burnFlash();
  }

  // private void logToDashboardInDetail() {
  //   SmartDashboard.putBoolean("Testing/" + name + "/ TurnInverted", mTurningMotor.getInverted());
    

  // }

  private void logToDashboard() {
    // SmartDashboard.putNumber("Drive/Module: " + name + ". Drive Output", mDriveMotor.getMotorOutputPercent());
    
    // SmartDashboard.putNumber("Drive/Module: " + name + ". Distance: ", getDistance());
    // SmartDashboard.putNumber("Drive/Module: " + name + ". Speed: ", getSpeed());

    // SmartDashboard.putNumber("Drive/Module: " + name + ". Angle: ", readAngle());
    // if (SmartDashboard.getBoolean("Drive/Resetting Module Values", false)) {
    //     SmartDashboard.putNumber("Drive/Module: " + name + ". Raw Angle: ", readAngle() + mAngleOffset);
    // }
    // SmartDashboard.putNumber("Testing/" + name + "/ Distance Traveled", mDriveMotor.getSelectedSensorPosition() * Constants.Drivebase.DISTANCE_PER_REVOLUTION);
    // SmartDashboard.putString("CANCoder Units", mTurningEncoder.getLastUnitString());
  }

  @Override
  public void periodic() {
    // if (Robot.detailedLogging) {
    //   logToDashboardInDetail();
    // }
    if (Robot.logging) {
      logToDashboard();
    }    
  // System.out.println("SwerveModule");
  
  }
}