// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Drive;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Drivebase;

public class DriveFalconOnly extends CommandBase {
  private final Drivebase mDrivebase;

  private final DoubleSupplier mTranslationXSupplier;


  /** Creates a new DriveFalconOnly. */
  public DriveFalconOnly(Drivebase drivebase, DoubleSupplier translationXSupplier) {
        mDrivebase = drivebase;
        mTranslationXSupplier = translationXSupplier;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mDrivebase.setFalconPercentOutput(mTranslationXSupplier.getAsDouble());

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mDrivebase.setFalconPercentOutput(0.0);

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
