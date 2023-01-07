// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.BallState;
import frc.robot.subsystems.tracking.LimelightInterface;

public class SpitBadBallOutShooter extends CommandBase {

  private Shooter mShooter;
  //private LimelightInterface mLimelightInterface;
  private boolean mIsManual;
  private double rpm;
  private double calculatedSpeed;
  // private double rpmThreshold;
  private double rpmBoost = Constants.Shooter.RPM_BOOST;
  private Tower mTower;

  private Tower.BallState mTeamState;

  int mBallLeavingCount;

  /**
   * Spits out other team's balls through the shooter
   * @param shooter
   * @param tower
   * @param teamColor what team we are on
   */
  public SpitBadBallOutShooter(Shooter shooter, Tower tower,
      Tower.BallState teamColor) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mTower = tower;
    mBallLeavingCount = 0;
    mTeamState = teamColor;
    // addRequirements(shooter, tower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (mTower.getTopBallState() != mTeamState && mTower.getTopBallState() != BallState.Empty) {

      rpm = Constants.Shooter.BAD_BALL_RPM;

      mShooter.setShooterPIDCorrection(rpm + rpmBoost);
      mTower.run();
      if (mTower.getBottomBeamBreak()) {
        mBallLeavingCount++;
      }
      if (mBallLeavingCount > Constants.FloorIntake.BALL_LEAVING_COUNT) {

        mBallLeavingCount = 0;
        mTower.mQueue[0] = mTower.mQueue[1]; // Updates queue when balls leave
        mTower.mQueue[1] = BallState.Empty; 
        mTower.ballsInRobot -= 2; // Because it also adds one within the tower subsystem
        mTower.ballsInRobot = Math.max(mTower.ballsInRobot, 0);

      }
      // System.out.println("Shooter");
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setSpeed(0.0);
    mTower.setSpeed(0.0);
    // mTower.closeHardStop();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return mBallLeavingCount >= Constants.FloorIntake.BALL_LEAVING_COUNT ||
        mTower.getTopBallState() == mTeamState ||
        mTower.getTopBallState() == BallState.Empty;
    // return false;
  }
}
