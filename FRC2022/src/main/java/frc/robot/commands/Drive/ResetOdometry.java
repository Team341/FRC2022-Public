// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;
;

public class ResetOdometry extends CommandBase {

  Drivebase mDrivebase;

  /**
   * Resets odometry
   * @param drivebase
   */
  public ResetOdometry(Drivebase drivebase) {
    mDrivebase = drivebase;
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivebase.resetOdometry(new Pose2d());
    mDrivebase.resetDriveModulePosition();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
