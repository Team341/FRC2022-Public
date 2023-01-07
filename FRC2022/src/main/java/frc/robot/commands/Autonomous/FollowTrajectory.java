// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivebase;

public class FollowTrajectory extends CommandBase {

  Drivebase mDrivebase;
  PathPlannerTrajectory mTrajectory;
  SequentialCommandGroup mSwerveControllerCommand;
  String mTrajectoryName;
  boolean mOptimized;
  boolean isFirstPath;
  private final Timer m_timer = new Timer();

  /** Creates a new FollowTrajectory. */
  public FollowTrajectory(Drivebase drivebase, PathPlannerTrajectory trajectory, String trajectoryName,
      boolean optimized, boolean isFirstPath) {
    this.mDrivebase = drivebase;
    this.mTrajectory = trajectory;
    this.mTrajectoryName = trajectoryName;
    this.mOptimized = optimized;
    this.isFirstPath = isFirstPath;
  }

  public FollowTrajectory(Drivebase drivebase, PathPlannerTrajectory trajectory, String trajectoryName,
      boolean isFirstPath) {
    this(drivebase, trajectory, trajectoryName, true, isFirstPath);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

    // if (!mOptimized) {
    // mDrivebase.getThetaController().disableContinuousInput();
    // }
    // else {
    // mDrivebase.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    // }
    // mDrivebase.getThetaController().setConstraints(Constants.Auto.kThetaControllerAutoConstraints);
    mSwerveControllerCommand = new SequentialCommandGroup(

        new InstantCommand(() -> {
          // Reset odometry for the first path you run during auto
          if (isFirstPath) {
            mDrivebase.resetOdometry(mTrajectory.getInitialHolonomicPose());
          }
        }),
        new PPSwerveControllerCommand(
            mTrajectory,
            mDrivebase::getPose, // Pose supplier
            mDrivebase.getKinematics(), // SwerveDriveKinematics
            new PIDController(
                SmartDashboard.getNumber("Auto/auto_kp", Constants.Drivebase.AUTO_PID_GAINS.kP),
                Constants.Drivebase.AUTO_PID_GAINS.kI,
                SmartDashboard.getNumber("Auto/auto_kd", Constants.Drivebase.AUTO_PID_GAINS.kD)),
            // Y Controller
            new PIDController(
                SmartDashboard.getNumber("Auto/auto_kp", Constants.Drivebase.AUTO_PID_GAINS.kP),
                Constants.Drivebase.AUTO_PID_GAINS.kI,
                SmartDashboard.getNumber("Auto/auto_kd", Constants.Drivebase.AUTO_PID_GAINS.kD)),
            // Turn controller
            mDrivebase.getThetaController(),
            mDrivebase::setModuleStates,
            mDrivebase)
      );

    mSwerveControllerCommand.initialize();

    // Are these necessary/helpful?
    // Is there some way to move these to only be run if detailedLogging is enabled?
    m_timer.reset();
    m_timer.start();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // Run the autonomous command
    mSwerveControllerCommand.execute();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mSwerveControllerCommand.end(interrupted);
    mDrivebase.getThetaController().enableContinuousInput(-Math.PI, Math.PI);
    mDrivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mSwerveControllerCommand.isFinished();
  }

  public void logToDashboard() {
    if (Robot.logging) {
      SmartDashboard.putString("Running Trajectory: ", mTrajectoryName);
      SmartDashboard.putNumber("Current Trajectory Time", m_timer.get());
      SmartDashboard.putNumber("Total Trajectory Time", mTrajectory.getTotalTimeSeconds());
    }
  }
}
