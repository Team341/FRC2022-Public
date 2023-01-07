// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Shooter;

public class SetShooterSpeed extends CommandBase {
  private Shooter mShooter;
  private double mSpeed;

  /**
   * Sets speed of shooter
   * @param shooter
   * @param speed motorspeed from -1.0 to 1.0
   */
  public SetShooterSpeed(Shooter shooter, double speed) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mSpeed = speed;
    addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setSpeed(mSpeed);
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
