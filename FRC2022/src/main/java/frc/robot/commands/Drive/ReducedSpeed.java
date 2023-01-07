// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.subsystems.Drivebase;

public class ReducedSpeed extends CommandBase {

  private final Drivebase mDrivebase;

  private final DoubleSupplier mTranslationXSupplier;
  private final DoubleSupplier mTranslationYSupplier;
  private final DoubleSupplier mRotationSupplier;

  /**
   * Drives in quarter the speed
   * @param drivebase 
   * @param translationXSupplier (meters per second)
   * @param translationYSupplier (meters per second)
   * @param rotationSupplier (Radians per second)
   */
  public ReducedSpeed(Drivebase drivebase,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {


    mDrivebase = drivebase;

    mTranslationXSupplier = translationXSupplier;
    mTranslationYSupplier = translationYSupplier;
    mRotationSupplier = rotationSupplier;

    addRequirements(mDrivebase);

    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    if (Robot.blueRobot) {
      mDrivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            mTranslationXSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mTranslationYSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mRotationSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mDrivebase.getAngle()));
    } else {
      mDrivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            mTranslationXSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mTranslationYSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mRotationSupplier.getAsDouble() / Constants.Drivebase.SPEED_REDUCTION_FACTOR,
            mDrivebase.getAngle()));
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivebase.drive(new ChassisSpeeds(0.0, 0.0, 0.0));
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
