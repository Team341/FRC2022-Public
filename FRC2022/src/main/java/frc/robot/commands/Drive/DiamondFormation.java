// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DiamondFormation extends CommandBase {

  private final Drivebase mDrivebase;

  /** Creates a new DiamondFormation. */
  public DiamondFormation(Drivebase drivebase) {
    mDrivebase = drivebase;

    addRequirements(drivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mDrivebase.drive(
    //     ChassisSpeeds.fromFieldRelativeSpeeds(
    //         0.0,
    //         0.0001,
    //         0.0,
    //         mDrivebase.getAngle()));

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { // Prevents robot from budging by setting wheels 45 degrees inwards
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(-45.0));
    SwerveModuleState sms2 = new SwerveModuleState(0.0, Rotation2d.fromDegrees(45.0));

    mDrivebase.setModuleStates(new SwerveModuleState[]{sms2, sms, sms, sms2});
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    SwerveModuleState sms = new SwerveModuleState(0.0, Rotation2d.fromDegrees(0.0));

    mDrivebase.setModuleStates(new SwerveModuleState[]{sms, sms, sms, sms});}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return mLeftX.getAsDouble() != 0.0 && mLeftY.getAsDouble() != 0.0;
    return false;
  }
}
