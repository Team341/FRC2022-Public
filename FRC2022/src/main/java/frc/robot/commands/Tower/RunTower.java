// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Tower;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Tower;

public class RunTower extends CommandBase {

  private Tower mTower;

  /**
   * Activates tower
   * @param tower
   */
  public RunTower(Tower tower) {
    mTower = tower;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mTower.closeHardStop();
    mTower.setSpeed(1.0);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() { //TODO tower should always be at 1?
    // if (mTower.colorSensorBallDetected()) {
    //   mTower.setSpeed(0.0);
    // } else {
      mTower.run();
    // }
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
