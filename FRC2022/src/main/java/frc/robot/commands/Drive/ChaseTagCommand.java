
package frc.robot.commands.Drive;

import static frc.robot.Constants.Vision.CAMERA_TO_ROBOT;

import java.util.function.Supplier;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.tracking.PhotonVisionInterface;

public class ChaseTagCommand extends CommandBase {

  private static final int TAG_TO_CHASE = 2;
  private static final Transform2d TAG_TO_GOAL = new Transform2d(new Translation2d(1, 0),
      Rotation2d.fromDegrees(180.0));

  private final PhotonCamera photonCamera;
  private final Drivebase drivetrainSubsystem;

  private final PIDController xController = new PIDController(
      SmartDashboard.getNumber("Auto/auto_kp", Constants.Drivebase.AUTO_PID_GAINS.kP),
      Constants.Drivebase.AUTO_PID_GAINS.kI,
      SmartDashboard.getNumber("Auto/auto_kd", Constants.Drivebase.AUTO_PID_GAINS.kD));

  private final PIDController yController = new PIDController(
      SmartDashboard.getNumber("Auto/auto_kp", Constants.Drivebase.AUTO_PID_GAINS.kP),
      Constants.Drivebase.AUTO_PID_GAINS.kI,
      SmartDashboard.getNumber("Auto/auto_kd", Constants.Drivebase.AUTO_PID_GAINS.kD));
  private PIDController omegaController;
  PhotonVisionInterface photon;
  private Pose2d goalPose;
  private PhotonTrackedTarget lastTarget;

  public ChaseTagCommand(
      PhotonVisionInterface photonCamera,
      Drivebase drivetrainSubsystem) {
    this.photonCamera = photonCamera.getCamera();
    this.drivetrainSubsystem = drivetrainSubsystem;
    omegaController = drivetrainSubsystem.getThetaController();

    xController.setTolerance(0.1);
    yController.setTolerance(0.1);
    photon = photonCamera;
    addRequirements(drivetrainSubsystem);
  }
  double xPos, yPos, theta;


  @Override
  public void initialize() {
    xPos = yPos = theta = 0.0;

    goalPose = null;
    lastTarget = null;
    omegaController.reset();
    xController.reset();
    yController.reset();
  }

  @Override
  public void execute() {
    double xSpeed, ySpeed;
    var robotPose = drivetrainSubsystem.getPose();
    var photonRes = photonCamera.getLatestResult();
    if (photonRes.hasTargets()) {
      // Find the tag we want to chase
      var targetOpt = photonRes.getTargets().stream()
          .filter(t -> t.getFiducialId() == TAG_TO_CHASE)
          .findFirst();
      if (targetOpt.isPresent()) {
        var target = targetOpt.get();

        if (!target.equals(lastTarget)) {
          drivetrainSubsystem.resetOdometry(robotPose);
          // This is new target data, so recalculate the goal
          lastTarget = target;

          // Get the transformation from the camera to the tag (in 2d)

          Translation2d translation = PhotonUtils.estimateCameraToTargetTranslation(
              photon.getDistance(), Rotation2d.fromDegrees(-target.getYaw()));

          xPos = translation.getX();
          yPos = translation.getY();


          theta = -Units.degreesToRadians(target.getYaw());


          SmartDashboard.putString("translation", translation.toString());

         

          // Transform the tag's pose to set our goal
        }

        //if (xPos != 0.0 || yPos != 0.0 || theta != 0.0) {
          // Drive
          xController.setSetpoint(xPos);
          yController.setSetpoint(yPos);
          SmartDashboard.putNumber("x chase", xPos);
          SmartDashboard.putNumber("y chase", yPos);
          SmartDashboard.putNumber("theta chase", theta);

          omegaController.setSetpoint(theta);
        //}
      }
    }

    xSpeed = xController.calculate(robotPose.getX());
    if (xController.atSetpoint() || !photonRes.hasTargets()) {
      xSpeed = 0;
    }

    ySpeed = yController.calculate(robotPose.getY());
    if (yController.atSetpoint() || !photonRes.hasTargets()) {
      ySpeed = 0;
    }

    SmartDashboard.putNumber("x speed val", xSpeed);
    SmartDashboard.putNumber("y speed val", ySpeed);
    SmartDashboard.putString("robot pose", robotPose.toString());

    var omegaSpeed = omegaController.calculate(robotPose.getRotation().getRadians());
    if (omegaController.atSetpoint() || !photonRes.hasTargets()) {
      omegaSpeed = 0;
    }

    drivetrainSubsystem.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, omegaSpeed, robotPose.getRotation()));
  }

  @Override
  public void end(boolean interrupted) {
    drivetrainSubsystem.drive(new ChassisSpeeds());
  }

}