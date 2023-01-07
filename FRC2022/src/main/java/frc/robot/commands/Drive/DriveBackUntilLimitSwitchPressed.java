// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.Drivebase;

public class DriveBackUntilLimitSwitchPressed extends CommandBase {
  Drivebase mDrivebase;
  ClimberHooks mClimberHooks;
  /** Creates a new DriveBackUntilLimitSwitchPressed. */
  public DriveBackUntilLimitSwitchPressed(Drivebase drivebase, ClimberHooks climberHooks) {
    mDrivebase = drivebase;
    mClimberHooks = climberHooks;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrivebase);
    addRequirements(mClimberHooks);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    double leftSpeed = Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND / 4.0;
    double rightSpeed = Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND / 4.0;

    if (mClimberHooks.getLeftLongLimitSwitch()) {
      leftSpeed = 0.0;
    }

    if (mClimberHooks.getRightLongLimitSwitch()) {
      rightSpeed = 0.0;
    }

    mDrivebase.driveTank(leftSpeed, rightSpeed);

    // mDrivebase.drive(
    //   new ChassisSpeeds(
    //     -1.0,
    //     0.0,
    //     0.0
    //   )
    // );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mClimberHooks.getLeftLongLimitSwitch() && mClimberHooks.getRightLongLimitSwitch();
  }
}
