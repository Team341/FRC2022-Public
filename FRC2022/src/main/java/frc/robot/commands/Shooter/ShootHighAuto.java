// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.Tower.BallState;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.subsystems.tracking.PhotonVisionInterface;

public class ShootHighAuto extends CommandBase {

  private Shooter mShooter;
  private PhotonVisionInterface mPhotonVisionInterface;
  private boolean mIsManual;
  private double rpm;
  private double calculatedSpeed;
  // private double rpmThreshold;
  private boolean mEnableLimelight;
  private double rpmBoost = Constants.Shooter.RPM_BOOST;
  private Tower mTower;
  private DoubleSupplier mDefaultRPM;

  private int count;

  private double time;
  private boolean hasTarget;

  private double addedRPM;

  private boolean end;
  private boolean hasFired;

  private boolean hasInitializedTimer = false;

  /**
   * Runs shoote
   * 
   * @param shooter
   * @param tower
   * @param enableLimelight whether limelight is enabled
   */
  public ShootHighAuto(Shooter shooter, Tower tower, boolean enableLimelight,
      DoubleSupplier defaultRPM, boolean end, double addedFirstRPM) {
    // Use addRequirements() here to declare subsystem dependencies.
    mShooter = shooter;
    mPhotonVisionInterface = PhotonVisionInterface.getInstance();
    rpm = Constants.Shooter.DEFAULT_HIGH_RPM;
    calculatedSpeed = 0.0;
    // rpmThreshold = 50.0;
    mIsManual = false;
    mEnableLimelight = enableLimelight;
    SmartDashboard.putNumber("Shooter/RPM Boost", rpmBoost);
    SmartDashboard.putBoolean("Shooter/Manual RPM Control", false);
    mTower = tower;

    mDefaultRPM = defaultRPM;
    this.end = end;
    hasInitializedTimer = false;
    addedRPM = addedFirstRPM;
  }

  public ShootHighAuto(Shooter shooter, Tower tower, boolean enableLimelight,
      DoubleSupplier defaultRPM, boolean end) {
    this(shooter, tower, enableLimelight, defaultRPM, true, 0.0);
  }

  public ShootHighAuto(Shooter shooter, Tower tower, boolean enableLimelight,
  DoubleSupplier defaultRPM) {
    this(shooter, tower, enableLimelight, defaultRPM, true);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    hasFired = false;
    hasInitializedTimer = false;

    count = 0;

    Constants.Shooter.SHOOTER_PID_GAINS.updateFromDashboard();
    mShooter.mTopShooterMotor.getPIDController().setP(Constants.Shooter.SHOOTER_PID_GAINS.kP);
    mShooter.mTopShooterMotor.getPIDController().setI(Constants.Shooter.SHOOTER_PID_GAINS.kI);
    mShooter.mTopShooterMotor.getPIDController().setD(Constants.Shooter.SHOOTER_PID_GAINS.kD);
    mShooter.mTopShooterMotor.getPIDController().setFF(Constants.Shooter.SHOOTER_PID_GAINS.kF);

    mShooter.mBottomShooterMotor.getPIDController().setP(-1.0 * Constants.Shooter.SHOOTER_PID_GAINS.kP);
    mShooter.mBottomShooterMotor.getPIDController().setI(Constants.Shooter.SHOOTER_PID_GAINS.kI);
    mShooter.mBottomShooterMotor.getPIDController().setD(Constants.Shooter.SHOOTER_PID_GAINS.kD);
    mShooter.mBottomShooterMotor.getPIDController().setFF(Constants.Shooter.SHOOTER_PID_GAINS.kF);

    mShooter.setIsShooting(true);
    // mTower.openHardStop();
    mIsManual = SmartDashboard.getBoolean("Shooter/Manual RPM Control", false);

    if (mIsManual) {
      rpm = SmartDashboard.getNumber("Shooter/RPM SetPoint", rpm);
    } else {
      rpm = mShooter.getRPMFromTableHigh(mPhotonVisionInterface.getDistance());
    }
    if (mEnableLimelight) {
      //mLimelightInterface.setLimeLightLED(3);
    }
    // rpmThreshold = SmartDashboard.getNumber("Shooter/RPM Threshold",
    // rpmThreshold);

    rpmBoost = SmartDashboard.getNumber("Shooter/RPM Boost", rpmBoost);
    // mShooter.setShooterPIDCorrection(rpm + rpmBoost);
    hasTarget = mPhotonVisionInterface.hasTarget();
    if (!hasTarget) {
      rpm = mDefaultRPM.getAsDouble();
    } else {
      rpm = mShooter.getRPMFromTableHigh(mPhotonVisionInterface.getDistance());
    }
    // logToDashboard();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if (!hasInitializedTimer) {
      time = Timer.getFPGATimestamp();
      hasInitializedTimer = true;
    }

    if (Timer.getFPGATimestamp() - time < 0.3) {
      mShooter.setSpeed(-0.3);
      if (Timer.getFPGATimestamp() - time < 0.25) {
        mTower.runBackwards();
      }
    } else {
      // if (mTower.getBeamBreak()) {
      //   count = 0;
      // }
      if (mShooter.getOnRPMTarget() || count > 3) {
        count++;
      } else {
        count = 0;
      }
      if (count > 3) {
        if (!mTower.getTopBeamBreak()) {
          mTower.run();
        } else {
          hasFired = true;
          count = 0;
          mTower.setSpeed(0.0);
        }
      } else {
        mTower.setSpeed(0.0);
      } 

      if (mIsManual) {
        rpm = SmartDashboard.getNumber("Shooter/RPM SetPoint", rpm);
      } else if (!hasTarget && mPhotonVisionInterface.hasTarget()) {
        hasTarget = true;
        rpm = mShooter.getRPMFromTableHigh(mPhotonVisionInterface.getDistance());
      }


      if (mEnableLimelight) {
       // mLimelightInterface.setLimeLightLED(3);
      }
      // rpmThreshold = SmartDashboard.getNumber("Shooter/RPM Threshold",
      // rpmThreshold);

      // mTower.run();
      if (!hasFired) {
        rpmBoost = addedRPM;
      } else {
        rpmBoost = 0.0;
      }
      mShooter.setShooterPIDCorrection(rpm + rpmBoost);
    }

    // logToDashboard();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mShooter.setSpeed(0.0);
    mTower.setSpeed(0.0);
    // mLimelightInterface.setLimeLightLED(1);
    // mTower.closeHardStop();
    mShooter.setIsShooting(false);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return mTower.getTopBallState() == BallState.Empty && mTower.getBottomBallState() == BallState.Empty;
    return count > 45 && end;

  }

  private void logToDashboard() {
    SmartDashboard.putNumber("Shooter/Calculated Speed", calculatedSpeed);
    SmartDashboard.putNumber("Shooter/RPM in Command", rpm);

  }
}