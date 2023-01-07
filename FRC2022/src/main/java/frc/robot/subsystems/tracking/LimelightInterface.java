package frc.robot.subsystems.tracking;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;

public class LimelightInterface extends SubsystemBase {
  private static LimelightInterface mInstance;

  public static LimelightInterface getInstance() {
    if (mInstance == null)
      mInstance = new LimelightInterface();
    return mInstance;
  }

  public double tv; // velocity
  public double tx; // x position
  public double ty; // y position
  public double ta; // acceleration
  private double ts;
  private double tl;
  private double tshort;
  private double tlong;
  private double thor;
  private double tvert;
  private double distanceToGoal;
  private double angleToGoal;
  private int angleOnGoalCount;

  public boolean lockedOnGoal;

  public static PIDController mLimelightController = new PIDController(
    Constants.Vision.VISION_PID_GAINS.kP,
    Constants.Vision.VISION_PID_GAINS.kI,
    Constants.Vision.VISION_PID_GAINS.kD);;
  public SimpleMotorFeedforward mTurnFeedforward;

  public LimelightInterface() {
    tv = 0.0;
    tx = 0.0;
    ty = 0.0;
    ta = 0.0;
    tlong = 0.0;
    tshort = 0.0;
    distanceToGoal = 0.0;
    angleToGoal = 0.0;
    angleOnGoalCount = 0;

    mTurnFeedforward = new SimpleMotorFeedforward(Constants.Drivebase.TURN_kS,
        Constants.Drivebase.VISION_KF);
    if (Robot.logging) {
      SmartDashboard.putNumber("Vision/kP", mLimelightController.getP());
      SmartDashboard.putNumber("Vision/kI", mLimelightController.getI());
      SmartDashboard.putNumber("Vision/kD", mLimelightController.getD());

    }

    
  }

  /**
   * This is the method for pulling values from LimeLight
   * This is called in the Robot Periodic Method
   */
  public void run() {
    tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
    tx = -1.0 * NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
    ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);
    tlong = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tlong").getDouble(0);
    tshort = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tshort").getDouble(0);
    calculateDistance();
    calculateAngle();
    // logToDashBoard();

    if (Math.abs(tx) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE) {
      angleOnGoalCount++;
    } else {
      angleOnGoalCount = 0;
    }
  }

  /**
   * Calculates the Distance of the robot from the goal
   */
  public void calculateDistance() {
    // double w = 320.0;
    // double a = 59.6;
    // double b = 49.7;
    // double distMultiplier = 1.0 / 10.0;
    distanceToGoal = ((Constants.Vision.GOAL_HEIGHT - Constants.Vision.LIMELIGHT_HEIGHT)
        / Math.tan(Math.toRadians(ty + Constants.Vision.INITIAL_ANGLE))) * 12.0;
  }

  /**
   * Calculates the angle of the robot from the goal
   */
  public void calculateAngle() {
    angleToGoal = tx;
  }

  /**
   * Sets the LED State of the LimeLight
   * 
   * @param ledState - 1: force off 2: force blink 3: force on
   */
  public void setLimeLightLED(int ledState) {
    NetworkTableInstance.getDefault().getTable("limelight").getEntry("ledMode").setNumber(ledState);
  }

  /**
   * Gets the already calculated distance from the goal without updating it
   * 
   * @return distance
   */
  public double getDistance() {
    return distanceToGoal;
  }

  public boolean tooClose() {
    return distanceToGoal <= Constants.Shooter.MINIMUM_SHOOTING_DISTANCE;
  }

  public boolean tooFar() {
    return distanceToGoal >= Constants.Shooter.MAXIMUM_SHOOTING_DISTANCE;
  }

  /**
   * Method that logs values to dashboard
   */
  public void logToDashBoard() {
    // SmartDashboard.putNumber("Vision/tv", tv);
    // SmartDashboard.putNumber("Vision/tx", tx);
    // mLimelightController.setP(SmartDashboard.getNumber("Vision/kP", mLimelightController.getP()));
    // mLimelightController.setI(SmartDashboard.getNumber("Vision/kI", mLimelightController.getI()));
    // mLimelightController.setD(SmartDashboard.getNumber("Vision/kD", mLimelightController.getD()));

    // SmartDashboard.putNumber("Vision/ty", ty);
    // SmartDashboard.putNumber("Vision/ta", ta);
    // SmartDashboard.putNumber("Vision/tlong", tlong);
    SmartDashboard.putNumber("Vision/Distance To Goal", distanceToGoal);
    // SmartDashboard.putNumber("Vision/Angle To Goal", angleToGoal);
  }

  /**
   * returns if there is a target detected by the limelight
   */
  public boolean hasTarget() {
    return tv > 0;
  }

  /**
   * returns if the robot is aligned with the target
   */
  public boolean alignedToGoal() {
    return hasTarget() && Math.abs(tx) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE && angleOnGoalCount >= 7;
  }

  public boolean notAligned() {
    return Math.abs(tx) > 5.5;
  }

}
