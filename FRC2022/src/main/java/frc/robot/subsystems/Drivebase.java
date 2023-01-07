// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.kauailabs.navx.IMUProtocol.GyroUpdate;
import com.kauailabs.navx.frc.AHRS;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.math.MatBuilder;
import edu.wpi.first.math.Nat;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.subsystems.tracking.PhotonVisionInterface;

public class Drivebase extends SubsystemBase {

  private static Drivebase instance = null;

  public static Drivebase getInstance() {
    if (instance == null) {
      instance = new Drivebase();
    }
    return instance;
  }

  private SwerveDriveKinematics mKinematics;

  private final AHRS mGyro = new AHRS(SPI.Port.kMXP);

  // These are our modules. We initialize them in the constructor.
  private final SwerveModule mFrontLeftModule;
  private final SwerveModule mFrontRightModule;
  private final SwerveModule mBackLeftModule;
  private final SwerveModule mBackRightModule;

  private ChassisSpeeds mChassisSpeeds = new ChassisSpeeds(0.0, 0.0, 0.0);

  private SwerveDriveOdometry mOdometry;

  private TrajectoryConfig mTrajectoryConfig;

  private PIDController mThetaController;

  public final PIDController mTurnController;
  public final SimpleMotorFeedforward mTurningFeedForward;
  public double AngleToOffset = 0.0;

  SwerveDrivePoseEstimator swerve;
  /** Creates a new Drivebase. */
  public Drivebase() {
    zeroGyroscope();

    mKinematics = new SwerveDriveKinematics(
        // Front left
        new Translation2d(Constants.Drivebase.DRIVEBASE_TRACKWIDTH_METERS / 2.0,
            Constants.Drivebase.DRIVEBASE_WHEELBASE_METERS / 2.0),
        // Front right
        new Translation2d(Constants.Drivebase.DRIVEBASE_TRACKWIDTH_METERS / 2.0,
            -Constants.Drivebase.DRIVEBASE_WHEELBASE_METERS / 2.0),
        // Back left
        new Translation2d(-Constants.Drivebase.DRIVEBASE_TRACKWIDTH_METERS / 2.0,
            Constants.Drivebase.DRIVEBASE_WHEELBASE_METERS / 2.0),
        // Back right
        new Translation2d(-Constants.Drivebase.DRIVEBASE_TRACKWIDTH_METERS / 2.0,
            -Constants.Drivebase.DRIVEBASE_WHEELBASE_METERS / 2.0));

    mFrontLeftModule = new SwerveModule(
        Constants.Drivebase.FRONT_LEFT_DRIVE_PORT,
        Constants.Drivebase.FRONT_LEFT_TURN_PORT,
        Constants.Drivebase.FRONT_LEFT_TURN_ENCODER_PORT,
        Robot.blueRobot ? Constants.Drivebase.BLUE_FRONT_LEFT_ANGLE_OFFSET
            : Constants.Drivebase.SILVER_FRONT_LEFT_ANGLE_OFFSET);

    mFrontLeftModule.setName("Front Left");
    mFrontRightModule = new SwerveModule(
        Constants.Drivebase.FRONT_RIGHT_DRIVE_PORT,
        Constants.Drivebase.FRONT_RIGHT_TURN_PORT,
        Constants.Drivebase.FRONT_RIGHT_TURN_ENCODER_PORT,
        Robot.blueRobot ? Constants.Drivebase.BLUE_FRONT_RIGHT_ANGLE_OFFSET
            : Constants.Drivebase.SILVER_FRONT_RIGHT_ANGLE_OFFSET);

    mFrontRightModule.setName("Front Right");

    mBackLeftModule = new SwerveModule(
        Constants.Drivebase.BACK_LEFT_DRIVE_PORT,
        Constants.Drivebase.BACK_LEFT_TURN_PORT,
        Constants.Drivebase.BACK_LEFT_TURN_ENCODER_PORT,
        Robot.blueRobot ? Constants.Drivebase.BLUE_BACK_LEFT_ANGLE_OFFSET
            : Constants.Drivebase.SILVER_BACK_LEFT_ANGLE_OFFSET);

    mBackLeftModule.setName("Back Left");

    mBackRightModule = new SwerveModule(
        Constants.Drivebase.BACK_RIGHT_DRIVE_PORT,
        Constants.Drivebase.BACK_RIGHT_TURN_PORT,
        Constants.Drivebase.BACK_RIGHT_TURN_ENCODER_PORT,
        Robot.blueRobot ? Constants.Drivebase.BLUE_BACK_RIGHT_ANGLE_OFFSET
            : Constants.Drivebase.SILVER_BACK_RIGHT_ANGLE_OFFSET);

    mBackRightModule.setName("Back Right");

    mOdometry = new SwerveDriveOdometry(mKinematics, getAngle());

    // Create config for trajectory
    mTrajectoryConfig = new TrajectoryConfig(
        Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        Constants.Drivebase.MAX_ACCELERATION)
            // Add kinematics to ensure max speed is actually obeyed
            .setKinematics(getKinematics());

    mThetaController = new PIDController(
        Constants.Drivebase.THETA_CONTROLLER_GAINS.kP,
        Constants.Drivebase.THETA_CONTROLLER_GAINS.kI,
        Constants.Drivebase.THETA_CONTROLLER_GAINS.kD);
    mThetaController.enableContinuousInput(-Math.PI, Math.PI);

    mTurnController = new PIDController(
        Constants.Drivebase.TURN_BODY_PID_GAINS.kP,
        Constants.Drivebase.TURN_BODY_PID_GAINS.kI,
        Constants.Drivebase.TURN_BODY_PID_GAINS.kD);
    mTurnController.enableContinuousInput(-180.0, 180.0);
    mTurnController.setTolerance(.1);

    mTurningFeedForward = new SimpleMotorFeedforward(
        Constants.Drivebase.kS,
        Constants.Drivebase.kV,
        Constants.Drivebase.kA);

    swerve = new SwerveDrivePoseEstimator(mGyro.getRotation2d(), 
    getPose(), 
    getKinematics(), 
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.02, 0.02, 0.01), // State measurement standard deviations. X, Y, theta.
    new MatBuilder<>(Nat.N1(), Nat.N1()).fill( 0.01), // Local measurement standard deviations. Left encoder, right encoder, gyro.
    new MatBuilder<>(Nat.N3(), Nat.N1()).fill(0.1, 0.1, 0.01)); // Global measurement standard deviations. X, Y, and theta.;
  

  }

  /**
   * @param chassisSpeeds sets designated states for each swerve module
   */
  public void drive(ChassisSpeeds chassisSpeeds) {
    mChassisSpeeds = chassisSpeeds;

    SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds);
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND);

    setModuleStates(states);
  }

  /**
   * Drives the robot as though it is tank, with 0 rotation
   * 
   * @param leftSpeed  speed in meters per second
   * @param rightSpeed speed in meters per second
   */
  public void driveTank(double leftSpeed, double rightSpeed) {
    SwerveModuleState frontLeftState = new SwerveModuleState(leftSpeed, Rotation2d.fromDegrees(0.0));
    SwerveModuleState backLeftState = new SwerveModuleState(leftSpeed, Rotation2d.fromDegrees(0.0));

    SwerveModuleState frontRightState = new SwerveModuleState(rightSpeed, Rotation2d.fromDegrees(0.0));
    SwerveModuleState backRightState = new SwerveModuleState(rightSpeed, Rotation2d.fromDegrees(0.0));
    SwerveModuleState[] states = { frontLeftState, frontRightState, backLeftState, backRightState };

    setModuleStates(states);
  }

  /**
   * Drives in a circle around given radius as if that was the new robot center
   * 
   * @param chassisSpeeds designated states for each swerve module
   * @param radius        radius of offset in meters
   */
  public void driveOffset(ChassisSpeeds chassisSpeeds, double radius) {
    mChassisSpeeds = chassisSpeeds;

    SwerveModuleState[] states = mKinematics.toSwerveModuleStates(mChassisSpeeds, new Translation2d(radius, 0.0));
    SwerveDriveKinematics.desaturateWheelSpeeds(states, Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND);

    setModuleStates(states);
  }

  public void setAllDriveMode(NeutralMode mode) {
    mFrontLeftModule.setIdleMode(mode);
    mFrontRightModule.setIdleMode(mode);
    mBackLeftModule.setIdleMode(mode);
    mBackRightModule.setIdleMode(mode);
  }

  public void setFalconPercentOutput(double output) {
    mFrontLeftModule.setFalconPercentOutput(output);
    mBackLeftModule.setFalconPercentOutput(output);
    mBackRightModule.setFalconPercentOutput(output);
    mFrontRightModule.setFalconPercentOutput(output);

  }

  /**
   * @return angle in degrees
   */
  public Rotation2d getAngle() {
    return Rotation2d.fromDegrees(-1.0 * mGyro.getAngle());
  }

  /**
   * @return current kinematics
   */
  public SwerveDriveKinematics getKinematics() {
    return mKinematics;
  }

  /**
   * @return current pose
   */
  public Pose2d getPose() {
    return mOdometry.getPoseMeters();
  }

  /**
   * @return theta controller
   */
  public PIDController getThetaController() {
    return mThetaController;
  }

  /**
   * @return trajectory config
   */
  public TrajectoryConfig getTrajectoryConfig() {
    return mTrajectoryConfig;
  }

  /**
   * Resets odometry given current pose
   * 
   * @param pose robot's current pose
   */
  public void resetOdometry(Pose2d pose) {
    mOdometry.resetPosition(pose, getAngle());
  }

  /**
   * Sets new states for each swerve module
   * 
   * @param states desired Swerve Module States for each swerve module
   */
  public void setModuleStates(SwerveModuleState[] states) {
    // updateOdometry(states);

    // SmartDashboard.putNumber("Testing/Front left angle",
    // states[0].angle.getDegrees());
    // SmartDashboard.putNumber("Testing/Front left compare to front right",
    // states[0].compareTo(states[1]));

    mFrontLeftModule.setDesiredState(states[0]);
    mFrontRightModule.setDesiredState(states[1]);
    mBackLeftModule.setDesiredState(states[2]);
    mBackRightModule.setDesiredState(states[3]);
  }

  /**
   * Sets new kinematics
   * 
   * @param kinematics
   */
  public void setKinematics(SwerveDriveKinematics kinematics) {
    mKinematics = kinematics;
  }

  /**
   * Offsets gyro by angle given
   * 
   * @param angle angle offset in degrees
   */
  public void offsetGyro(double angle) {
    mGyro.setAngleAdjustment(angle);
  }
  
 
  /**
   * Updates robot's position on the field
   */
  public void updateOdometry() {
    // System.out.println("Updating Odometry");
    mOdometry.update(
        getAngle(),
        mFrontLeftModule.getState(),
        mFrontRightModule.getState(),
        mBackLeftModule.getState(),
        mBackRightModule.getState());

        double imageCaptureTime = Timer.getFPGATimestamp();
          swerve.updateWithTime(imageCaptureTime,mGyro.getRotation2d(),mFrontLeftModule.getState(),mFrontRightModule.getState(),mBackLeftModule.getState(),mBackRightModule.getState());
          var res = PhotonVisionInterface.getInstance().getCamera().getLatestResult();
          if (res.hasTargets()) {
            Transform3d camToTargetTrans = res.getBestTarget().getBestCameraToTarget();
            SmartDashboard.putString("camtotarget", camToTargetTrans.toString());

            Transform2d a = new Transform2d(new Translation2d(camToTargetTrans.getX(), camToTargetTrans.getY()), camToTargetTrans.getRotation().toRotation2d());
            if (PhotonVisionInterface.getPoseFromID(res.getBestTarget().getFiducialId()) != null) {
            Pose2d camPose =PhotonVisionInterface.getPoseFromID(res.getBestTarget().getFiducialId()).transformBy(a.inverse());
            SmartDashboard.putString("cam pose", camPose.toString());
            swerve.addVisionMeasurement(camPose.transformBy(Constants.Vision.CAMERA_TO_ROBOT), imageCaptureTime);
            SmartDashboard.putString("cam transformed pose", camPose.transformBy(Constants.Vision.CAMERA_TO_ROBOT).toString());
            SmartDashboard.putNumber("Image Capture time", imageCaptureTime);
            }
            
          }
        
        SmartDashboard.putString("pose estimate", swerve.getEstimatedPosition().toString());
    
    
        // SwerveModuleState[] states =
    // mKinematics.toSwerveModuleStates(mChassisSpeeds);
    // SwerveDriveKinematics.desaturateWheelSpeeds(states,
    // Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND);
    // mOdometry.update(\
    // getAngle(),
    // states
    // );
  }

  /**
   * Resets all the swerve module's encoders
   */
  public void resetDriveModulePosition() {
    mFrontLeftModule.resetDriveEncoder();
    mFrontRightModule.resetDriveEncoder();
    mBackLeftModule.resetDriveEncoder();
    mBackRightModule.resetDriveEncoder();
  }

  public void invertTurnModules(boolean isInverted) {
    mFrontLeftModule.setTurnMotorInverted(isInverted);
    mFrontRightModule.setTurnMotorInverted(isInverted);
    mBackLeftModule.setTurnMotorInverted(isInverted);
    mBackRightModule.setTurnMotorInverted(isInverted);
  }

  public void setBrakeMode(boolean breakMode) {
    mFrontLeftModule.setBrakeMode(breakMode);
    mBackLeftModule.setBrakeMode(breakMode);
    mFrontRightModule.setBrakeMode(breakMode);
    mBackRightModule.setBrakeMode(breakMode);
  }

  /**
   * Sets the gyroscope angle to zero. This can be used to set the direction the
   * robot is currently facing to the
   * 'forwards' direction.
   */
  public void zeroGyroscope() {
    mGyro.reset();
    mGyro.setAngleAdjustment(0.0);
  }

  private void logToDashboard() {
    SmartDashboard.putNumber("Drive/X translation", Units.metersToInches(mOdometry.getPoseMeters().getX()));
    // SmartDashboard.putNumber("Drive/Odometry Reported Heading",
    // mOdometry.getPoseMeters().getRotation().getDegrees());
    SmartDashboard.putNumber("Drive/Y translation", Units.metersToInches(mOdometry.getPoseMeters().getY()));
    SmartDashboard.putNumber("Drive/Heading", getAngle().getDegrees());

    // SmartDashboard.putNumber("Drive/Drive Speed X",
    // mChassisSpeeds.vxMetersPerSecond);
    // SmartDashboard.putNumber("Drive/Drive Speed Y",
    // mChassisSpeeds.vyMetersPerSecond);
  }

  private void logToDashboardInDetail() {
    SmartDashboard.putNumber("Drive/Angle in Radians", mOdometry.getPoseMeters().getRotation().getRadians());
    SmartDashboard.putNumber("Drive/Angle in Degrees", mOdometry.getPoseMeters().getRotation().getDegrees());
  }

  @Override
  public void periodic() {

    updateOdometry();

    if (Robot.detailedLogging) {
      logToDashboardInDetail();
    }
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("Drivebase");
  }
  

}