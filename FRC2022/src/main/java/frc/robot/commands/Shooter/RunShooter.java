// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class RunShooter extends CommandBase {
  Shooter mShooter;
  DoubleSupplier mLeftY;

  /**
   * Activates the shooter
   * @param shooter
   * @param leftY joystick input between -1.0 and 1.0
   */
  public RunShooter(Shooter shooter, DoubleSupplier leftY) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mLeftY = leftY;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setSpeed(mLeftY.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
