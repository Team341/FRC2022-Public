// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.utilities.DaisyMath;

public class ShooterRPMSetPoint extends CommandBase {
  Shooter mShooter;
  double rpm;

  /**
   * Setpoint for rotations per minute of shooter
   * @param shooter
   * @param rpm rotations per minute
   */
  public ShooterRPMSetPoint(Shooter shooter, double rpm) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.mShooter = shooter;
    this.rpm = rpm;
    // addRequirements(mShooter);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mShooter.setIsShooting(true);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mShooter.setShooterPIDCorrection(rpm);
    // mShooter.setSpeed(DaisyMath.minmax(Constants.Shooter.SHOOTER_PID_GAINS.kF * rpm, 0.0, 1.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setSpeed(0.0);
    mShooter.setIsShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
