// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

public class FieldOrientedDrive extends CommandBase {

  private final Drivebase mDrivebase;

  private final DoubleSupplier mTranslationXSupplier;
  private final DoubleSupplier mTranslationYSupplier;
  private final DoubleSupplier mRotationSupplier;

  // private final SlewRateLimiter mXLimiter = new SlewRateLimiter(20.0);
  // private final SlewRateLimiter mYLimiter = new SlewRateLimiter(20.0);

  /**
   * Creates a new FieldOrientedDrive 
   * @param drivebase
   * @param translationXSupplier (meters per second)
   * @param translationYSupplier (meters per second)
   * @param rotationSupplier (radians persecond)
   */
  public FieldOrientedDrive(Drivebase drivebase, 
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplier) {
    
    mDrivebase = drivebase;
    mTranslationXSupplier = translationXSupplier;
    mTranslationYSupplier = translationYSupplier;
    mRotationSupplier = rotationSupplier;

    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mXLimiter.reset(0.0);
    // mYLimiter.reset(0.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    SmartDashboard.putNumber("ChassisInputX", mTranslationXSupplier.getAsDouble());
    SmartDashboard.putNumber("ChassisInputY", mTranslationYSupplier.getAsDouble());
    SmartDashboard.putNumber("ChassisInputR", mRotationSupplier.getAsDouble());
    
    
    Double tx = mTranslationXSupplier.getAsDouble();
    Double ty = mTranslationYSupplier.getAsDouble();
    Double td = Math.hypot(tx, ty);

    if (td <= Constants.ControllerInputs.DEADBAND * 1.0) {
      tx = (tx/Math.max(td, 0.001))*Constants.ControllerInputs.DEADBAND / 10.0;
      ty = (ty/Math.max(td, 0.001))*Constants.ControllerInputs.DEADBAND / 10.0;

      // tx *= .01;
      // ty *= .01;
      // tx = 0.0;
      // ty = 0.0;

    }
    // else {
    //   tx *= tx * tx;
    //   ty *= ty * ty;
    // }
    // tx = mXLimiter.calculate(tx);
    // ty = mYLimiter.calculate(ty);

    mDrivebase.drive(
            ChassisSpeeds.fromFieldRelativeSpeeds(
                tx,
                ty,
                mRotationSupplier.getAsDouble(),
                mDrivebase.getAngle()));


    // mDrivebase.drive(
    //   new ChassisSpeeds(
    //       mTranslationXSupplier.getAsDouble(),
    //       mTranslationYSupplier.getAsDouble(),
    //       mRotationSupplier.getAsDouble()));

          
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
