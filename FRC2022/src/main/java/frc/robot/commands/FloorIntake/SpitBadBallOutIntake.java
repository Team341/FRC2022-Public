/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.FloorIntake;

import java.util.concurrent.BlockingQueue;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.BallState;

public class SpitBadBallOutIntake extends CommandBase {
  private FloorIntake mFloorIntake;
  BallState mTeamState;
  Tower mTower;
  int mBallLeavingCount;

  /**
   * Gets rid of other team's balls through intake
   * @param floor
   * @param tower
   * @param teamState What team we are on
   */
  public SpitBadBallOutIntake(FloorIntake floor, Tower tower, BallState teamState) {
    // Use addRequirements() here to declare subsystem dependencies.
    mFloorIntake = floor;
    mTower = tower;
    mTeamState = teamState;
    // addRequirements(mFloorIntake, tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if (mTower.getBottomBallState() != mTeamState) {
      // mFloorIntake.deployIntake();
    }
    mBallLeavingCount = 0;

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // SmartDashboard.putNumber("Ball Leaving Count", mBallLeavingCount);
    mFloorIntake.reverseIntake();
    mTower.runBackwards();
    if (mTower.getBottomBeamBreak()) {
      mBallLeavingCount++;
    }
    if (mBallLeavingCount > Constants.FloorIntake.BALL_LEAVING_COUNT) {
      mBallLeavingCount = 0;
      mTower.mQueue[1] = mTower.mQueue[0]; // Updates queue when balls leave
      mTower.mQueue[0] = BallState.Empty;
      mTower.ballsInRobot -= 2; // Because it also adds one within the tower subsystem
      mTower.ballsInRobot = Math.max(mTower.ballsInRobot, 0); 

    }
    // System.out.println("Intake");
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mTower.setSpeed(0.0);
    mFloorIntake.setIntakeSpeed(0.0);
    // mFloorIntake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mTower.getBottomBallState() == mTeamState ||
      mTower.getBottomBallState() == BallState.Empty;
    // return false;
  }
}