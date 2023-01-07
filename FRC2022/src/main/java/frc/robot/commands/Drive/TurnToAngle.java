// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.utilities.DaisyMath;

// 1640 do this here:https://github.com/FRC1640/2021-Code/blob/main/src/main/java/frc/robot/autonomous/commands/AlignAuton.java

public class TurnToAngle extends CommandBase {

  private Drivebase mDrivebase;
  private DoubleSupplier mLeftY;
  private DoubleSupplier mLeftX;
 
  // PID Values for turn
  // private double kP = Constants.Drivebase.ROTATE_PID_GAINS.kP;
  // private double kI = Constants.Drivebase.ROTATE_PID_GAINS.kI;
  // private double kD = Constants.Drivebase.ROTATE_PID_GAINS.kD;
  // private double kF = Constants.Drivebase.VISION_KF;

  private double mTargetAngle;
  // private double maxOutput;

  private double error;

  /**
   * Original align to goal
   * @param drivebase
   * @param leftY left joystick y
   * @param leftX right joystick x 
   */
  public TurnToAngle(Drivebase drivebase, Double targetAngle) {
    // Use addRequirements() here to declare subsystem dependencies.
    mDrivebase = drivebase;
    // maxOutput = 0.35;
    mTargetAngle = targetAngle;
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    //Is the below right? If this isn't driving correctly, I would expect it to be because of this
    // tx is the error
    // Should this be in the execute?
    // targetAngle = DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() - mLimelightInterface.tx);
    // mDrivebase.getThetaController().setGoal(targetAngle);
    // targetAngle = DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() + mLimelightInterface.tx);
    // mLimelightInterface.mLimelightController.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error = DaisyMath.boundAngleNeg180to180Degrees(mTargetAngle - mDrivebase.getAngle().getDegrees());
    double rot = Constants.Drivebase.VISION_KF * error / 2.0;//DaisyMath.boundAngleNeg180to180Degrees(mLimelightInterface.mLimelightController.getSetpoint() - mDrivebase.getAngle().getDegrees()));

    // SmartDashboard.putNumber("Vision/FF Input", targetAngle - mDrivebase.getAngle().getDegrees());
    // SmartDashboard.putNumber("Vision/Target Angle", targetAngle);
    // SmartDashboard.putNumber("Vision/Angle When Calculating", mDrivebase.getAngle().getDegrees());

    // I see that instead of multiplying by max angular velocity, 1640 just multiplies by 2, might that be better? Nah.
    // double rot = Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * // Does this need to be multiplied by the max speed?
    //     // mDrivebase.getThetaController().calculate(
    //     mLimelightInterface.mLimelightController.calculate(
    //       mDrivebase.getAngle().getDegrees()
    //     );
    // rot = 0.0;

    mDrivebase.drive(
      ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0,/* rot +*/ rot, mDrivebase.getAngle())
    );
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivebase.drive(new ChassisSpeeds());
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(
    //   DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() - mLimelightInterface.tx)
    //   ) < Constants.Drivebase.VISION_ANGLE_TOLERANCE;
    // return mDrivebase.getThetaController().atGoal();
    // return mLimelightInterface.alignedToGoal() || !mLimelightInterface.hasTarget();
    return Math.abs(error) < Constants.Drivebase.VISION_ANGLE_TOLERANCE;
  }
}