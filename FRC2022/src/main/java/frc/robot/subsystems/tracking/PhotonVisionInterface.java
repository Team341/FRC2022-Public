// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.tracking;

import java.util.HashMap;

import com.kauailabs.navx.frc.AHRS;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class PhotonVisionInterface extends SubsystemBase {

  private PhotonCamera camera = new PhotonCamera("Microsoft_LifeCam_HD-3000 (1)");
  private PhotonPipelineResult result = new PhotonPipelineResult();
  private double angleOnGoalCount = 0.;
  private double distanceToGoal, angleToGoal;
  private Drivebase mDrivebase;
  private static HashMap<Integer, Pose2d> mp = new HashMap<>();
  private Pose2d robotPose = new Pose2d();
  public boolean lockedOnGoal = false;

  private static PhotonVisionInterface mInstance;

  public static PhotonVisionInterface getInstance() {
    if (mInstance == null)
      mInstance = new PhotonVisionInterface(Drivebase.getInstance());
    return mInstance;
  }


  /** Creates a new PhotonVisionInterface. */
  public PhotonVisionInterface(Drivebase drivebase) {
    mDrivebase = drivebase;
    
    mp.put(5, new Pose2d(new Translation2d(Units.feetToMeters(24.75), Units.feetToMeters(16)), new Rotation2d()));
    mp.put(3, new Pose2d(new Translation2d(Units.feetToMeters(12.33), Units.feetToMeters(21.2)), new Rotation2d()));
    mp.put(1, new Pose2d(new Translation2d(Units.feetToMeters(-1.66), Units.feetToMeters(11)), new Rotation2d()));
    mp.put(2, new Pose2d(new Translation2d(Units.feetToMeters(8), Units.feetToMeters(0)), new Rotation2d()));
    mp.put(4, new Pose2d(new Translation2d(Units.feetToMeters(15), Units.feetToMeters(0)), new Rotation2d()));
    mp.put(0, new Pose2d(new Translation2d(Units.feetToMeters(20.66), Units.feetToMeters(3.2)), new Rotation2d()));
  }

  public static Pose2d getPoseFromID( int ID) {
    return mp.get(ID);
  }

  public Pose2d getRobotPoseEstimate() {
    return robotPose;
  }
  /**
   * This is the method for pulling values from LimeLight
   * This is called in the Robot Periodic Method
   */
  public void run() {
    result = camera.getLatestResult();
    if (result.hasTargets()) {
      double tx = result.getBestTarget().getYaw();
      calculateDistance();
      calculateAngle();
      updatePoseEstimate();
      if (Math.abs(tx) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE) {
        angleOnGoalCount++;
      } 
      else {
        angleOnGoalCount = 0;
      }
    // logToDashBoard();
    }
    else {
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
    if (result.hasTargets()) {
      SmartDashboard.putNumber("angle pitch", Units.degreesToRadians(result.getBestTarget().getPitch()));
      distanceToGoal = PhotonUtils.calculateDistanceToTargetMeters(
          Constants.Vision.LIMELIGHT_HEIGHT_METER,
          Constants.Vision.GOAL_HEIGHT_METER,
          Rotation2d.fromDegrees(Constants.Vision.INITIAL_ANGLE).getRadians(),
          Units.degreesToRadians(result.getBestTarget().getPitch()));

    }
  }


  /**
   * Calculates the angle of the robot from the goal
   */
  public void calculateAngle() {
    angleToGoal = result.getBestTarget().getYaw();
  }

  /**
   * Gets the already calculated distance from the goal without updating it
   * 
   * @return distance
   */
  public double getDistance() {
    return Units.metersToInches(distanceToGoal);
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
    // mLimelightController.setP(SmartDashboard.getNumber("Vision/kP",
    // mLimelightController.getP()));
    // mLimelightController.setI(SmartDashboard.getNumber("Vision/kI",
    // mLimelightController.getI()));
    // mLimelightController.setD(SmartDashboard.getNumber("Vision/kD",
    // mLimelightController.getD()));

    // SmartDashboard.putNumber("Vision/ty", ty);
    // SmartDashboard.putNumber("Vision/ta", ta);
    // SmartDashboard.putNumber("Vision/tlong", tlong);
    SmartDashboard.putNumber("Vision/Distance To Goal", distanceToGoal);
    SmartDashboard.putNumber("Vision/Goal height", Constants.Vision.GOAL_HEIGHT_METER);
    SmartDashboard.putNumber("Vision/Limelight height", Constants.Vision.LIMELIGHT_HEIGHT_METER);

    if (result.hasTargets()) {
      SmartDashboard.putNumber("Vision/April Tag Id", result.getBestTarget().getFiducialId());

      SmartDashboard.putString("Vision/Pose Estimate", mp.getOrDefault(result.getBestTarget().getFiducialId(), new Pose2d()).toString());
    }

    // SmartDashboard.putNumber("Vision/Angle To Goal", angleToGoal);
  }

  /**
   * returns if there is a target detected by the limelight
   */
  public boolean hasTarget() {
    return result.hasTargets();
  }

  /**
   * returns if the robot is aligned with the target
   */
  public boolean alignedToGoal() {
    return hasTarget() && Math.abs(result.getBestTarget().getYaw()) <= Constants.Drivebase.VISION_ANGLE_TOLERANCE
        && angleOnGoalCount >= 7;
  }

  public boolean notAligned() {
    return Math.abs(result.getBestTarget().getYaw()) > 5.5;
  }

  public void updatePoseEstimate() {
    
    // Calculate robot's field relative pose
    robotPose = PhotonUtils.estimateFieldToRobot(
        Constants.Vision.LIMELIGHT_HEIGHT_METER, Constants.Vision.GOAL_HEIGHT_METER, Rotation2d.fromDegrees(Constants.Vision.INITIAL_ANGLE).getRadians(), result.getBestTarget().getPitch(), Rotation2d.fromDegrees(-result.getBestTarget().getYaw()),
        mDrivebase.getAngle(), mp.getOrDefault(result.getBestTarget().getFiducialId(), new Pose2d()), Constants.Vision.CAMERA_TO_ROBOT);
  }


  public PhotonCamera getCamera() {
    return camera;
  }

}
