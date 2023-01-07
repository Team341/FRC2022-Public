// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.LED;

import java.util.function.BooleanSupplier;

import com.ctre.phoenix.led.ColorFlowAnimation;
import com.ctre.phoenix.led.RainbowAnimation;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.BallState;
import frc.robot.subsystems.tracking.PhotonVisionInterface;
import frc.robot.subsystems.Shooter.firingState;

// TODO this isn't on it's own thread by default, figure that out!

public class RunLED extends CommandBase {
  Tower mTower;
  LED mLED;
  Shooter mShooter;
  PhotonVisionInterface mPhotonVisionInterface;
  private BallState mBottomBallState;
  private BallState mTopBallState;
  private firingState mFiringState;
  private ClimberHooks mClimberHooks;
  private Climber mClimber;
  BooleanSupplier mAlign;

  boolean hasBeenAligned;

  /**
   * Updates LED based on balls in tower and shooting state
   * 
   * @param led
   * @param tower
   * @param shooter
   * @param limelight
   */
  public RunLED(LED led, Tower tower, Shooter shooter, PhotonVisionInterface limelight, ClimberHooks climberHooks,
      Climber climber, BooleanSupplier aligning) {
    mTower = tower;
    mLED = led;
    mShooter = shooter;
    mPhotonVisionInterface = limelight;
    mBottomBallState = BallState.Empty;
    mTopBallState = BallState.Empty;
    mFiringState = firingState.NotOnTargetOrRPM;

    mClimberHooks = climberHooks;
    mClimber = climber;
    mAlign = aligning;
      
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mLED);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!mAlign.getAsBoolean()) {
      mPhotonVisionInterface.lockedOnGoal = false;
    }
    if (mClimber.mClimbing) {
      // SmartDashboard.putString("LOgging gray", "CLIMBING");
      // if (mClimberHooks.getLeftShortLimitSwitch()) {
      // mLED.setLeftShortLEDs(Color.kGreen);
      // } else {
      // mLED.setLeftShortLEDs(Color.kRed);
      // }

      // if (mClimberHooks.getLeftLongLimitSwitch()) {
      // mLED.setLeftLongLEDs(Color.kGreen);
      // } else {
      // mLED.setLeftLongLEDs(Color.kRed);
      // }

      // if (mClimberHooks.getRightShortLimitSwitch()) {
      // mLED.setRightShortLEDs(Color.kGreen);
      // } else {
      // mLED.setRightShortLEDs(Color.kRed);
      // }

      // if (mClimberHooks.getRightLongLimitSwitch()) {
      // mLED.setRightLongLEDs(Color.kGreen);
      // } else {
      // mLED.setRightLongLEDs(Color.kRed);
      // }
      if (Climber.rainbowClimbing) {
        mLED.getLED()
            .animate(new RainbowAnimation(1.0, 1.0, Constants.LED.LEFT_LED_LENGTH + Constants.LED.RIGHT_LED_LENGTH));
      } else {
        if (Climber.climbing) {
          mLED.getLED().setLEDs(0, 255, 0);
        } else {
          mLED.getLED().setLEDs(255, 0, 0);
        }
      }

    } else {
      // if (mShooter.isShooting()) {
      // // if (SmartDashboard.getBoolean("Shooter/Is Shooting", false)) { // TODO
      // replace with real shooter
      // mFiringState = firingState.Shooting;
      // // mLED.getLED().animate(new RainbowAnimation(1.0, 1.0, 55)); // Rainbow when
      // shooting
      // // mLED.setBottomLEDColor(Constants.LED.IS_SHOOTING_COLOR);
      // // mLED.setTopLEDColor(Constants.LED.IS_SHOOTING_COLOR);
      // // mLED.setFiringLEDColor(Constants.LED.IS_SHOOTING_COLOR);
      // mLED.getLED().setLEDs((int) Constants.LED.IS_SHOOTING_COLOR.red * 255, (int)
      // Constants.LED.IS_SHOOTING_COLOR.green * 255, (int)
      // Constants.LED.IS_SHOOTING_COLOR.blue * 255);
      /* } else */
      if (mPhotonVisionInterface.lockedOnGoal || mLED.isUsingAutoLED()) {
        // SmartDashboard.putString("Logging Gray", "ALIGNED");
        // if (mShooter.getOnRPMTarget()) {
        //   mFiringState = firingState.OnTargetAndRPM;
        //   mLED.getLED().setLEDs((int) Constants.LED.ON_TARGET_AND_RPM_COLOR.red * 255,
        //       (int) Constants.LED.ON_TARGET_AND_RPM_COLOR.green * 255,
        //       (int) Constants.LED.ON_TARGET_AND_RPM_COLOR.blue * 255);
        //   // mLED.setFiringLEDColor(Constants.LED.ON_TARGET_AND_RPM_COLOR);
        // } else {
        //   mFiringState = firingState.OnTargetNotOnRPM;
        //   mLED.getLED().setLEDs((int) Constants.LED.ON_TARGET_NOT_ON_RPM_COLOR.red * 255,
        //       (int) Constants.LED.ON_TARGET_NOT_ON_RPM_COLOR.green * 255,
        //       (int) Constants.LED.ON_TARGET_NOT_ON_RPM_COLOR.blue * 255);
        //   // mLED.setFiringLEDColor(Constants.LED.ON_TARGET_NOT_ON_RPM_COLOR);
        if (mPhotonVisionInterface.hasTarget()) { 
          if (mPhotonVisionInterface.tooClose()) {
            mLED.getLED().setLEDs(0, 0, 255);
          }
          else if (mPhotonVisionInterface.tooFar()) {
            mLED.getLED().setLEDs(255, 0, 0);
          }
          else {
            if (mPhotonVisionInterface.notAligned()) {
              hasBeenAligned = false;
            }
            if (mPhotonVisionInterface.alignedToGoal() || hasBeenAligned) {
              hasBeenAligned = true;
            } 
            if (mPhotonVisionInterface.lockedOnGoal || hasBeenAligned) {
              mLED.getLED().setLEDs(0, 255, 0);
            } else {
              mLED.getLED().setLEDs(0, 0, 0);
            }
          }
        } else {
          mLED.getLED().setLEDs(255, 0, 0);
        }
        // }
       }
      //  else {
      //    mLED.getLED().setLEDs(0, 0, 0);
      //  } 
       else {
        // SmartDashboard.putString("Logging Gray", "YES");
        if (true) {
          // Both balls are correct collor
          if (mTower.mQueue[0] == BallState.Blue && mTower.mQueue[1] == BallState.Blue) {
            mLED.getLED().setLEDs((int) Color.kYellow.red * 255, (int) Color.kYellow.green * 255, (int) Color.kYellow.blue * 255);
          }
          // Either ball is wrong color
          // else if (mTower.mQueue[0] == BallState.Red && mTower.mQueue[1] == BallState.Red) {
          //   // mLED.getLED().setLEDs(255, 0, 0);
          //   mLED.getLED().setLEDs((int) Color.kYellow.red * 255, (int) Color.kYellow.green * 255, (int) Color.kYellow.blue * 255);

          // }
          else if (mTower.mQueue[0] == BallState.Blue || mTower.mQueue[1] == BallState.Blue) {
            // mLED.getLED().setLEDs((int) Color.kPurple.red * 255, (int) Color.kPurple.green * 255, (int) Color.kPurple.blue * 255);
            mLED.getLED().setLEDs(255, 0, 255);
          }
          // Neither ball is wrong color, but one is empty
          else {
            mLED.getLED().setLEDs((int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.red * 255,
                (int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.green * 255,
                (int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.blue * 255);
          }
        } 
        // else {
        //   // Both balls are correct collor
        //   if (mTower.mQueue[0] == BallState.Red && mTower.mQueue[1] == BallState.Red) {
        //     mLED.getLED().setLEDs((int) Color.kYellow.red * 255, (int) Color.kYellow.green * 255, (int) Color.kYellow.blue * 255);
        //   }
        //   // Either ball is wrong color
        //   else if (mTower.mQueue[0] == BallState.Blue || mTower.mQueue[1] == BallState.Blue) {
        //     mLED.getLED().setLEDs(255, 0, 0);
        //   }
        //   // Neither ball is wrong color, but one is empty
        //   else {
        //     mLED.getLED().setLEDs((int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.red * 255,
        //         (int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.green * 255,
        //         (int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.blue * 255);
        //   }
        // }
        // else {
        // mLED.getLED().setLEDs((int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.red *
        // 255, (int) Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.green * 255, (int)
        // Constants.LED.NOT_ON_TARGET_OR_RPM_COLOR.blue * 255);
      }
    
    // } else {
    // switch (mTower.getTopBallState()) { // Sets top LED based on ball color

    // case Empty:

    // mTopBallState = BallState.Empty;
    // mLED.setTopLEDColor(Constants.LED.EMPTY_COLOR);
    // break;
    // case Red:
    // mTopBallState = BallState.Red;
    // mLED.setTopLEDColor(Color.kRed);
    // break;
    // case Blue:
    // mTopBallState = BallState.Blue;
    // mLED.setTopLEDColor(Color.kBlue);
    // break;
    // }

    // switch (mTower.getBottomBallState()) { // Sets bottom LED based on ball color
    // case Empty:
    // mBottomBallState = BallState.Empty;
    // mLED.setBottomLEDColor(Constants.LED.EMPTY_COLOR);
    // break;
    // case Red:
    // mBottomBallState = BallState.Red;
    // mLED.setBottomLEDColor(Color.kRed);
    // break;
    // case Blue:
    // mBottomBallState = BallState.Blue;
    // mLED.setBottomLEDColor(Color.kBlue);
    // break;
    // }

    // }
    // }

    // SmartDashboard.putString("LED/Bottom State", mBottomBallState.toString());
    // SmartDashboard.putString("LED/Top State", mTopBallState.toString());
    // SmartDashboard.putString("LED/Firing State", mFiringState.toString());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
