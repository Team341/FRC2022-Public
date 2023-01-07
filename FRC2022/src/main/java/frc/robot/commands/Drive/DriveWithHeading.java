// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utilities.DaisyMath;

public class DriveWithHeading extends CommandBase {

  private final Drivebase mDrivebase;

  private final DoubleSupplier mTranslationXSupplier;
  private final DoubleSupplier mTranslationYSupplier;
  private final DoubleSupplier mRotationSupplierX;
  private final DoubleSupplier mRotationSupplierY;
  private final SlewRateLimiter mSlewRateLimiter;

  private double lastDirection;

  /** Creates a new DriveWithHeading. */
  public DriveWithHeading(Drivebase drivebase,
      DoubleSupplier translationXSupplier,
      DoubleSupplier translationYSupplier,
      DoubleSupplier rotationSupplierX,
      DoubleSupplier rotationSupplierY) {
    mDrivebase = drivebase;

    lastDirection = 0.0;

    mSlewRateLimiter = new SlewRateLimiter(Constants.Drivebase.TURN_SLEW_RATE);

    mTranslationXSupplier = translationXSupplier;
    mTranslationYSupplier = translationYSupplier;
    mRotationSupplierX = rotationSupplierX;
    mRotationSupplierY = rotationSupplierY;

    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    
    double tx = mTranslationXSupplier.getAsDouble();
    double ty = mTranslationYSupplier.getAsDouble();
    double rx = mRotationSupplierX.getAsDouble();
    double ry = mRotationSupplierY.getAsDouble();
    double td = Math.hypot(tx, ty);
    double rd = Math.hypot(rx, ry);

    // This is the inverse tangent of the x and the y of the turn controller
    // rx and ry are switched, as converting controller frame to robot frame results in a 90 degree offset
    // which this accounts for
    // may need to switch rx/ry
    double Setpoint = Units.radiansToDegrees(Math.atan2(rx, ry));

    // Ignore new setpoint, use old setpoint
    if (rd < Constants.ControllerInputs.DEADBAND) {
      Setpoint = lastDirection;
    } 
    // Store the last setpoint
    else {
      lastDirection = Setpoint;
    }
    mDrivebase.getThetaController().setSetpoint(Setpoint);
    
    SmartDashboard.putNumber("Drive/Angle Setpoint", Setpoint);
    mDrivebase.getThetaController().setP(SmartDashboard.getNumber("Drive/Drivebase Turn P", mDrivebase.getThetaController().getP()));
    mDrivebase.getThetaController().setD(SmartDashboard.getNumber("Drive/Drivebase Turn D", mDrivebase.getThetaController().getD()));

    SmartDashboard.putNumber("Drive/Position Error", mDrivebase.getThetaController().getPositionError());
    SmartDashboard.putNumber("Drive/Velocity Error", mDrivebase.getThetaController().getVelocityError());


    // Do the same thing with the translation
    if (td <= Constants.ControllerInputs.DEADBAND) {
      tx = (tx / Math.max(td, 0.001)) * Constants.ControllerInputs.DEADBAND * 0.5;
      ty = (ty / Math.max(td, 0.001)) * Constants.ControllerInputs.DEADBAND * 0.5;
    } else {
      tx *= tx * tx;
      ty *= ty * ty;
    }

    mDrivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(
            tx,
            ty,
            // Units.radiansToDegrees(
            // Math.atan2(
            // mRotationSupplierY.getAsDouble(),
            // mRotationSupplierX.getAsDouble()
            // )
            // ),
            // 0.0,
            mSlewRateLimiter.calculate(mDrivebase.getThetaController().calculate(mDrivebase.getAngle().getDegrees()) + mDrivebase.mTurningFeedForward.calculate(mDrivebase.getThetaController().getVelocityError())),
            mDrivebase.getAngle())
    // new ChassisSpeeds(

    // mTranslationXSupplier.getAsDouble(),
    // mTranslationYSupplier.getAsDouble(),
    // mDrivebase.mTurnController.calculate(mDrivebase.getAngle().getDegrees())
    // )
    );

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
