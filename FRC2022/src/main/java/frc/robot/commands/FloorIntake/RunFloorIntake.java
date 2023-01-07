/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.FloorIntake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Tower;

public class RunFloorIntake extends CommandBase {
  private FloorIntake mFloorIntake;
  private boolean mLoweredIntake;
  int mBallCount;
  Tower mTower;
  boolean mColorSensorBallDetected;
  int mBallDetectedCount;
  
  /**
   * Runs intake
   * @param floor
   * @param tower
   */
  public RunFloorIntake(FloorIntake floor, Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTower = tower;
    mFloorIntake = floor;


    // addRequirements(mFloorIntake, mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBallCount = mTower.getBallsInRobotCount();

    mBallDetectedCount = 0;
    // if (mBallCount < Constants.FloorIntake.MAX_BALL_COUNT) { // if there are already two balls in tower do not intake
    mFloorIntake.deployIntake();
    // }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    mFloorIntake.runIntake();
    mTower.run();
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

    mFloorIntake.setIntakeSpeed(0.0);
    // mFloorIntake.retractIntake();
    mTower.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}