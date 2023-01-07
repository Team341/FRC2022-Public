// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class TeleopHeading extends CommandBase {

  Drivebase mDrivebase;
  double mAngleOffset;

  /**
   * Updates teleop heading
   * @param drivebase
   * @param angleOffset (degrees)
   */
  public TeleopHeading(Drivebase drivebase, double angleOffset) {
    mDrivebase = drivebase;
    mAngleOffset = angleOffset;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mDrivebase.offsetGyro(mAngleOffset);
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
