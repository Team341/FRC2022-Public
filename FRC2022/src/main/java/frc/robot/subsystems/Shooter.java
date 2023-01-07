// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.io.BufferedReader;
import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.TreeMap;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class Shooter extends SubsystemBase {

  public static enum firingState {
    NotOnTargetOrRPM,
    OnTargetNotOnRPM,
    NotOnTargetOnRPM,
    OnTargetAndRPM,
    Shooting
  }

  /**
   * creates a new Shooter
   */
  private static Shooter instance = null;

  public static Shooter getInstance() {
    if (instance == null) {
      instance = new Shooter();
    }
    return instance;
  }

  public CANSparkMax mTopShooterMotor;
  public CANSparkMax mBottomShooterMotor;

  private CANCoder mShooterEncoder;

  private TreeMap<Double, Double> mRPMTableLow;
  private TreeMap<Double, Double> mRPMTableHigh;
  private double targetCount;
  private int offTargetCount;
  private double rpmThreshold;
  private double rpmSetPoint;
  private File fileLow;
  private File fileHigh;
  private ProfiledPIDController mShooterPIDController;
  private boolean mIsShooting = false;

  private static double differenceBetweenRPMAndGoal;

  private double MAX_RPM;
  

  public Shooter() {
    // Setup the shooter motors (2x Neo)
    mTopShooterMotor = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_PORT_TOP, MotorType.kBrushless); // TODO make
                                                                                                        // sure this
                                                                                                        // motor is
                                                                                                        // brushless
    mBottomShooterMotor = new CANSparkMax(Constants.Shooter.SHOOTER_MOTOR_PORT_BOTTOM, MotorType.kBrushless);

    // Configure the shooter motor settings
    mTopShooterMotor.restoreFactoryDefaults();
    mBottomShooterMotor.restoreFactoryDefaults();

    mTopShooterMotor.setSmartCurrentLimit(Constants.Shooter.SHOOTER_MOTOR_CURRENT_LIMIT);
    mBottomShooterMotor.setSmartCurrentLimit(Constants.Shooter.SHOOTER_MOTOR_CURRENT_LIMIT);

    mTopShooterMotor.setIdleMode(IdleMode.kCoast);
    mBottomShooterMotor.setIdleMode(IdleMode.kCoast);

    mTopShooterMotor.setOpenLoopRampRate(0.5);
    mBottomShooterMotor.setOpenLoopRampRate(0.5);

    // Restrict backwards motion since the shooter should only ever really spin in
    // the desired direction
    mTopShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    mTopShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    mBottomShooterMotor.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, true);
    mBottomShooterMotor.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, 0);

    mTopShooterMotor.setInverted(true);
    mBottomShooterMotor.setInverted(false);

    // Burn the configuration to flash memory to prevent issues if power is lost
    // while operating
    mTopShooterMotor.burnFlash();
    mBottomShooterMotor.burnFlash();

    // Create the CANCoder object which we'll be using to track the shooter's RPM
    // mShooterEncoder = new CANCoder(Constants.Shooter.CAN_ENCODER_PORT);

    // Default some of the internal shooter parameters
    rpmSetPoint = Constants.Shooter.DEFAULT_HIGH_RPM;
    rpmThreshold = Constants.Shooter.DEFAULT_RPM_THRESHOLD;

    targetCount = 0.0; // Counts how many cycles the shooter RPM has been on target
    offTargetCount = 0;

    // Creation of RPM table
    // TODO verify RPM table
    mRPMTableLow = new TreeMap<Double, Double>();
    mRPMTableHigh = new TreeMap<Double, Double>();

    // Get file location
    fileLow = new File("/home/lvuser/deploy/RangeTableLow.csv");
    fileHigh = new File("/home/lvuser/deploy/RangeTableHigh.csv");

    try {
      createRPMTable();
    } catch (IOException e) {
      e.printStackTrace();
    }

    SmartDashboard.putBoolean("Shooter/Manual RPM Control", false);
    SmartDashboard.putNumber("Shooter/RPM SetPoint", 0.0);

    Constants.Shooter.SHOOTER_PID_GAINS.logToDashboard();

    mShooterPIDController = new ProfiledPIDController(
      Constants.Shooter.SHOOTER_PID_GAINS.kP, 
      Constants.Shooter.SHOOTER_PID_GAINS.kI, 
      Constants.Shooter.SHOOTER_PID_GAINS.kD, 
      new Constraints(Constants.Shooter.MAX_SHOOTER_VELOCITY, Constants.Shooter.MAX_SHOOTER_ACCELERATION));

    mTopShooterMotor.getPIDController().setP(Constants.Shooter.SHOOTER_PID_GAINS.kP);
    mTopShooterMotor.getPIDController().setI(Constants.Shooter.SHOOTER_PID_GAINS.kI);
    mTopShooterMotor.getPIDController().setD(Constants.Shooter.SHOOTER_PID_GAINS.kD);
    mTopShooterMotor.getPIDController().setFF(Constants.Shooter.SHOOTER_PID_GAINS.kF);

    mBottomShooterMotor.getPIDController().setP(-1.0 * Constants.Shooter.SHOOTER_PID_GAINS.kP);
    mBottomShooterMotor.getPIDController().setI(Constants.Shooter.SHOOTER_PID_GAINS.kI);
    mBottomShooterMotor.getPIDController().setD(Constants.Shooter.SHOOTER_PID_GAINS.kD);
    mBottomShooterMotor.getPIDController().setFF(Constants.Shooter.SHOOTER_PID_GAINS.kF);

    mTopShooterMotor.getPIDController().setOutputRange(0.0, 1.0);
    mBottomShooterMotor.getPIDController().setOutputRange(0.0, 1.0);

    // mShooterPIDController = new ProfiledPIDController(
    // Constants.Shooter.SHOOTER_PID_GAINS.kP,
    // Constants.Shooter.SHOOTER_PID_GAINS.kI,
    // Constants.Shooter.SHOOTER_PID_GAINS.kD,
    // new TrapezoidProfile.Constraints(Constants.Shooter.MAX_SHOOTER_VELOCITY,
    // Constants.Shooter.MAX_SHOOTER_ACCELERATION));
    MAX_RPM = Robot.blueRobot ? Constants.Shooter.MAX_BLUE_RPM : Constants.Shooter.MAX_SILVER_RPM;

  }

  /**
   * sets speed of motors on shooter.
   * we only have to do this for the first motor as the second is slaved
   * 
   * @param motorSpeed between -1.0 to 1.0
   */
  public void setSpeed(double motorSpeed) {
    mTopShooterMotor.set(motorSpeed);
    mBottomShooterMotor.set(motorSpeed);

  }

  /**
   * Sets PID correction on shooter
   * 
   * @param goalRPM
   */
  public void setShooterPIDCorrection(double goalRPM) {
    setRPMSetPoint(goalRPM);
    // mTopShooterMotor.set(mShooterPIDController.calculate(getTopRPM(), goalRPM));
    // mBottomShooterMotor.set(mShooterPIDController.calculate(getBottomRPM(), goalRPM));
    mTopShooterMotor.getPIDController().setReference(goalRPM, ControlType.kVelocity);
    mBottomShooterMotor.getPIDController().setReference(goalRPM, ControlType.kVelocity);
  }

  /**
   * Method for getting shooter rpm
   * 
   * @return Current shooter RPM
   */
  public double getRPM() {
    return mTopShooterMotor.getEncoder().getVelocity() * Constants.Shooter.RPM_COEFFICIENT;
  }

  /**
   * Method for getting top shooter rpm
   * 
   * @return Current top shooter RPM
   */
  public double getTopRPM() {
    return mTopShooterMotor.getEncoder().getVelocity() * Constants.Shooter.RPM_COEFFICIENT;
  }

  /**
   * Method for getting bottom shooter rpm
   * 
   * @return Current bottom shooter RPM
   */
  public double getBottomRPM() {
    return mBottomShooterMotor.getEncoder().getVelocity() * Constants.Shooter.RPM_COEFFICIENT;
  }

  /**
   * @return the difference between the goal rpm and the current
   */
  public double getDifferenceBetweenRPMAndGoal() {
    return differenceBetweenRPMAndGoal;
  }

  /**
   * @param differenceBetweenRPMAndGoal updates the difference between the rpm and
   *                                    the goal
   */
  public void setDifferenceBetweenRPMAndGoal(double differenceBetweenRPMAndGoal) {
    Shooter.differenceBetweenRPMAndGoal = differenceBetweenRPMAndGoal;
  }

  /**
   * Gets whether or not the shooter is close to set rpm
   * 
   * @return true = at target rpm
   */
  public boolean getOnRPMTarget() {
    // rpmSetPoint = SmartDashboard.getNumber("Shooter/RPM in Command", rpmSetPoint);
    rpmThreshold = SmartDashboard.getNumber("Shooter/RPM Threshold", rpmThreshold);
    return Math.abs(rpmSetPoint - getRPM()) < rpmThreshold;
  }

  public void setRPMSetPoint(double setpoint) {
    rpmSetPoint = setpoint;
  }

  /**
   * Gets the count of being on target
   * 
   * @return target counter
   */
  public double getOnTargetCount() {
    return targetCount;
  }

  /**
   * Creates the Shooter RPM Table
   * 
   * @throws IOException
   */
  public void createRPMTable() throws IOException {
    String line = "";
    BufferedReader csvReader = new BufferedReader(new FileReader(fileLow));
    line = csvReader.readLine();
    while (line != null) {
      String[] temp = line.split(",");
      mRPMTableLow.put(Double.parseDouble(temp[0]), Double.parseDouble(temp[1]));
      line = csvReader.readLine();
    }
    csvReader.close();
    
    line = "";
    csvReader = new BufferedReader(new FileReader(fileHigh));
    line = csvReader.readLine();
    while (line != null) {
      String[] temp = line.split(",");
      mRPMTableHigh.put(Double.parseDouble(temp[0]), Double.parseDouble(temp[1]));
      line = csvReader.readLine();
    }
    csvReader.close();
  }

  /**
   * Gets the rpm table
   * 
   * @return RPM Table
   */
  public TreeMap<Double, Double> getRPMTable() {
    return mRPMTableLow;
  }

  /**
   * Gets the calculated RPM based on distance
   * 
   * @param range - distance from goal
   * @return RPM for shooter
   */
  public double getRPMFromTableLow(double range) {
    double lowKey = -1.0;
    double lowVal = -1.0;
    double returnVal = Constants.Shooter.DEFAULT_SHOOTER_RPM;

    for (double key : mRPMTableLow.keySet()) {
      if (range < key) {
        double highVal = mRPMTableLow.get(key);
        if (lowKey > 0.0) {
          double m = (range - lowKey) / (key - lowKey);
          returnVal = lowVal + m * (highVal - lowVal);
          break;
        } else {
          returnVal = highVal;
          break;
        }
      }
      lowKey = key;
      lowVal = mRPMTableLow.get(key);
    }
    return Math.min(returnVal, MAX_RPM);
  }

  /**
   * Gets the calculated RPM based on distance
   * 
   * @param range - distance from goal
   * @return RPM for shooter
   */
  public double getRPMFromTableHigh(double range) {
    double lowKey = -1.0;
    double lowVal = -1.0;
    double returnVal = Constants.Shooter.DEFAULT_SHOOTER_RPM;

    for (double key : mRPMTableHigh.keySet()) {
      if (range < key) {
        double highVal = mRPMTableHigh.get(key);
        if (lowKey > 0.0) {
          double m = (range - lowKey) / (key - lowKey);
          returnVal = lowVal + m * (highVal - lowVal);
          break;
        } else {
          returnVal = highVal;
          break;
        }
      }
      lowKey = key;
      lowVal = mRPMTableHigh.get(key);
    }
    return Math.min(returnVal, MAX_RPM);
  }

  /**
   * returns the primary encoder port of the shooter
   * 
   * @return the encoder for the first shooter motor
   */
  public RelativeEncoder getBuiltInEncoder() {
    return mTopShooterMotor.getEncoder();
  }

  /**
   * @return gets the value of the rpm threshold
   */
  public double getRPMThreshold() {
    return rpmThreshold;
  }

  public void setIsShooting(boolean isShooting) {
    mIsShooting = isShooting;
  }

  public boolean isShooting() {
    return mIsShooting;
  }

  // private void logToDashBoardInDetail() {
  //   SmartDashboard.putNumber("Shooter/RPM Threshold", rpmThreshold);
  //   SmartDashboard.putBoolean("Shooter/In RPM Range", getOnRPMTarget());
  //   SmartDashboard.putNumber("Shooter/On RPM Count", getOnTargetCount());
  //   SmartDashboard.putNumber("Shooter/Difference Between RPM And Goal", differenceBetweenRPMAndGoal);
  //   SmartDashboard.putBoolean("Shooter/Is Shooting", isShooting());

  // }

  /**
   * logs the motor's velocity to the dashboard
   */
  private void logToDashboard() {
    // SmartDashboard.putNumber("Shooter/Shooter RPM", getRPM());
    SmartDashboard.putNumber("Shooter/Top RPM", getTopRPM());
    SmartDashboard.putNumber("Shooter/Bottom RPM", getBottomRPM());
    // SmartDashboard.putNumber("Shooter/Error", rpmSetPoint - getRPM());

    // SmartDashboard.putString("Shooter/Get Last Unit String",
    // mShooterEncoder.getLastUnitString());

    boolean mIsManual = SmartDashboard.getBoolean("Shooter/Manual RPM Control", false);
    if (!mIsManual) {
      SmartDashboard.putNumber("Shooter/RPM SetPoint", rpmSetPoint);
    }
  }

  @Override
  public void periodic() {

    if (getOnRPMTarget()) {
      targetCount++;
      offTargetCount = 0;
    } else {
      if (offTargetCount > 5) {
        targetCount = 0.0;
      } else {
        offTargetCount++;
      }
    }

    // if (Robot.detailedLogging) {
    //   logToDashBoardInDetail();
    // }
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("Shooter");
  }
}
