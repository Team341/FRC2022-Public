// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Tower;

public class RunTowerBack extends CommandBase {

  private Tower mTower;
 
  /**
   * Reverses tower
   * @param tower
   */
  public RunTowerBack(Tower tower) {
    mTower = tower;
    // Use addRequirements() here to declare subsystem dependencies.
    // addRequirements(mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mTower.runBackwards();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mTower.runBackwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTower.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
