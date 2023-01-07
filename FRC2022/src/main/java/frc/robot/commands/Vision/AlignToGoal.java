// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import javax.lang.model.util.ElementScanner6;
import javax.xml.stream.events.StartElement;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.subsystems.tracking.PhotonVisionInterface;
import frc.robot.utilities.DaisyMath;

// 1640 do this here:https://github.com/FRC1640/2021-Code/blob/main/src/main/java/frc/robot/autonomous/commands/AlignAuton.java

public class AlignToGoal extends CommandBase {

  private PhotonVisionInterface photonVision;
  private Drivebase mDrivebase;
  private DoubleSupplier mLeftY;
  private DoubleSupplier mLeftX;
  // PID Values for turn
  // private double kP = Constants.Vision.VISION_PID_GAINS.kP;
  // private double kI = Constants.Vision.VISION_PID_GAINS.kI;
  // private double kD = Constants.Vision.VISION_PID_GAINS.kD;
  private double kF = Constants.Drivebase.VISION_KF;

  private double targetAngle;
  private double maxOutput;
  double filteredTx;
  private boolean isDone = false;

  private DoubleSupplier mTurn;

  /**
   * Original align to goal
   * 
   * @param drivebase
   * @param leftY     left joystick y
   * @param leftX     right joystick x
   */
  public AlignToGoal(Drivebase drivebase, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier turn) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonVision = PhotonVisionInterface.getInstance();

    mDrivebase = drivebase;
    mLeftY = leftY;
    mLeftX = leftX;
    targetAngle = 0.0;
    maxOutput = 0.35;
    
    // SmartDashboard.putNumber("Vision/kP", kP);
    // SmartDashboard.putNumber("Vision/kI", kI);
    // SmartDashboard.putNumber("Vision/kD", kD);
    SmartDashboard.putNumber("Vision/Max Turn Output", maxOutput);
    mTurn = turn;

    addRequirements(mDrivebase);

  }

  boolean mExitCondition;
  int state = 0; // 0 for searching, 1 for found, 2 for travelling

  int count;
  boolean hasBeenSeen = false;
  double PreviousYaw = 0.0;
  double gyroAngle = 0.0;

  /**
   * Original align to goal
   * 
   * @param drivebase
   * @param leftY         left joystick y
   * @param leftX         right joystick x
   * @param exitCondition
   */
  public AlignToGoal(Drivebase drivebase, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier turn,
      boolean exitCondition) {
    // Use addRequirements() here to declare subsystem dependencies.
    photonVision = PhotonVisionInterface.getInstance();

    mDrivebase = drivebase;
    mLeftY = leftY;
    mLeftX = leftX;
    targetAngle = 0.0;
    // maxOutput = 0.35;
    // SmartDashboard.putNumber("Vision/kP", kP);
    // SmartDashboard.putNumber("Vision/kI", kI);
    // SmartDashboard.putNumber("Vision/kD", kD);
    SmartDashboard.putNumber("Vision/Max Turn Output", maxOutput);
    mTurn = turn;

    mExitCondition = exitCondition;

    LimelightInterface.mLimelightController.enableContinuousInput(-Math.PI, Math.PI);
    LimelightInterface.mLimelightController.setTolerance(Math.toRadians(Constants.Drivebase.VISION_ANGLE_TOLERANCE));;

    addRequirements(mDrivebase);

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // SmartDashboard.putNumber("Vision/kP", kP);
    // SmartDashboard.putNumber("Vision/kI", kI);
    // SmartDashboard.putNumber("Vision/kD", kD);
    var result = photonVision.getCamera().getLatestResult();
    if (result.hasTargets()) {
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      filteredTx = Math.toRadians(result.getBestTarget().getYaw());
    } else {
      filteredTx = 0.0;
    }

    count = 0;
    hasBeenSeen = false;
    PreviousYaw = 0.0;
    gyroAngle = 0.0;
    LimelightInterface.mLimelightController.reset();
    // Is the below right? If this isn't driving correctly, I would expect it to be
    // because of this
    // tx is the error
    // Should this be in the execute?
    // targetAngle =
    // DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() -
    // mLimelightInterface.tx);
    // mDrivebase.getThetaController().setGoal(targetAngle);
    // targetAngle =
    // DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() +
    // mLimelightInterface.tx);
    // mLimelightInterface.mLimelightController.setSetpoint();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = mLeftY.getAsDouble();
    double ySpeed = mLeftX.getAsDouble();

    // filteredTx = (Constants.Vision.TX_FILTER * mLimelightInterface.tx)
    // + ((1 - Constants.Vision.TX_FILTER) * filteredTx);
    var result = photonVision.getCamera().getLatestResult();
    double rotationSpeed=0.0;

    if (count>=10) {
      if (result.hasTargets()) {
        if (LimelightInterface.mLimelightController.atSetpoint()) {
          isDone = true;
        }
        else {
          count = 0;
          hasBeenSeen = false;
        }
      }
      else {
        count = 0;
        hasBeenSeen = false;
      }
    }

    else if (hasBeenSeen){
      // If we have no targets, stay still.
      // Calculate angular turn power
      // -1.0 required to ensure positive PID controller effort _increases_ yaw
      rotationSpeed = LimelightInterface.mLimelightController.calculate(mDrivebase.getAngle().getRadians(), gyroAngle+PreviousYaw);
    
       if (LimelightInterface.mLimelightController.atSetpoint()) {
         count++;
       }
      double error = DaisyMath.boundAngleNegPiToPiRadians(filteredTx) * kF;
       state = 2;
      // rotationSpeed += error;
    } else if (result.hasTargets()) {
        filteredTx = (Constants.Vision.TX_FILTER *Math.toRadians(result.getBestTarget().getYaw()))
          + ((1 - Constants.Vision.TX_FILTER) * filteredTx);
        PreviousYaw = -Math.toRadians(result.getBestTarget().getYaw());
        gyroAngle = mDrivebase.getAngle().getRadians();
        state = 1;
  
  
        SmartDashboard.putNumber("Vision/yaw setpoint", result.getBestTarget().getYaw());
        
        // Calculate angular turn power
        // -1.0 required to ensure positive PID controller effort _increases_ yaw
        rotationSpeed = LimelightInterface.mLimelightController.calculate(mDrivebase.getAngle().getRadians(), gyroAngle+PreviousYaw);
  
        SmartDashboard.putString("Vision/cam3d", result.getBestTarget().getBestCameraToTarget().toString());
        
        if (LimelightInterface.mLimelightController.atSetpoint()) {
          count++;
        }
        double error = DaisyMath.boundAngleNegPiToPiRadians(filteredTx) * kF;
  
        // rotationSpeed += error;
        hasBeenSeen = true;
  
    }else {
      rotationSpeed = -0.5;
      hasBeenSeen = false;
      PreviousYaw = 0.0;
      state = 0;
    }

    
    
    SmartDashboard.putNumber("Vision/err", LimelightInterface.mLimelightController.getPositionError());
    SmartDashboard.putBoolean("Vision/at setpoint ", LimelightInterface.mLimelightController.atSetpoint());


    // double mAngle = -mDrivebase.getAngle().getRadians();
    // // double strafeComponent = xSpeed * Math.sin(mAngle) + ySpeed *
    // Math.cos(mAngle);
    // // double predictiveRot = Constants.Drivebase.PREDICTIVE_PID_GAINS.kF *
    // -strafeComponent;
    // // double angleOffset = Constants.Drivebase.PREDICTIVE_PID_GAINS.kF *
    // -strafeComponent;

    // // filteredTx = (Constants.Vision.TX_FILTER * (mLimelightInterface.tx + 3.0 *
    // angleOffset))
    // // + ((1 - Constants.Vision.TX_FILTER) * filteredTx);

    // filteredTx = (Constants.Vision.TX_FILTER * mLimelightInterface.tx)
    // + ((1 - Constants.Vision.TX_FILTER) * filteredTx);

    // // double rot = Constants.Drivebase.VISION_KF * error + kP * error;//
    // //
    // DaisyMath.boundAngleNeg180to180Degrees(mLimelightInterface.mLimelightController.getSetpoint()
    // // -
    // // mDrivebase.getAngle().getDegrees()));

    // // SmartDashboard.putNumber("Vision/Strafe Component", strafeComponent);
    SmartDashboard.putNumber("Vision/Rotation", rotationSpeed);
    SmartDashboard.putNumber("Vision/setpoint phi",   Math.toDegrees(gyroAngle+PreviousYaw )     );
    SmartDashboard.putNumber("Vision/setpoint PID",   Math.toDegrees(LimelightInterface.mLimelightController.getSetpoint() )     );

    SmartDashboard.putNumber("Vision/state",   state     );
    SmartDashboard.putNumber("Vision/count",   count     );



    
    // SmartDashboard.putNumber("Vision/Predictive Rotation", predictiveRot);

    // SmartDashboard.putNumber("Vision/FF Input", targetAngle -
    // mDrivebase.getAngle().getDegrees());
    // SmartDashboard.putNumber("Vision/Target Angle", targetAngle);
    // SmartDashboard.putNumber("Vision/Angle When Calculating",
    // mDrivebase.getAngle().getDegrees());

    // I see that instead of multiplying by max angular velocity, 1640 just
    // multiplies by 2, might that be better?
    // double rot = Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND * //
    // Does this need to be multiplied by the max speed?
    // // mDrivebase.getThetaController().calculate(
    // mLimelightInterface.mLimelightController.calculate(
    // mDrivebase.getAngle().getDegrees()
    // );
    // rot = 0.0;
    // if (mLimelightInterface.hasTarget()) {
    // if (mExitCondition) {
    // double positionError = 0.0;
    // if (mLimelightInterface.tooClose() || mLimelightInterface.tooFar()) {
    // positionError = Constants.Shooter.IDEAL_SHOOTER_DISTANCE -
    // mLimelightInterface.getDistance();
    // }
    // mDrivebase.drive(new ChassisSpeeds(-positionError *
    // Constants.Vision.DRIVE_VISION_PID_GAINS.kP, 0.0, rot));
    // SmartDashboard.putNumber("Position Error", -positionError *
    // Constants.Vision.DRIVE_VISION_PID_GAINS.kP);
    // } else {
    // mDrivebase.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, /* rot + */ rot +
    // predictiveRot,
    // mDrivebase.getAngle()));
    // }
    // } else {
    // rot = mTurn.getAsDouble();
    // mDrivebase.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, /* rot + */
    // mTurn.getAsDouble() + predictiveRot,
    // mDrivebase.getAngle()));
    // }
    mDrivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, /* rot + */ rotationSpeed,
            mDrivebase.getAngle()));

    // mDrivebase.drive(
    // ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, /* rot + */ rot +
    // predictiveRot,
    // mDrivebase.getAngle()));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    photonVision.lockedOnGoal = true;
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    
    // return Math.abs(
    // DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() -
    // mLimelightInterface.tx)
    // ) < Constants.Drivebase.VISION_ANGLE_TOLERANCE;
    // return mDrivebase.getThetaController().atGoal();
    // return mLimelightInterface.alignedToGoal() ||
    // !mLimelightInterface.hasTarget();
    return (isDone && count >= 10
        && mExitCondition && mLeftY.getAsDouble() == 0.0 && mLeftX.getAsDouble() == 0.0);// &&
                                                                                        // !mLimelightInterface.tooFar()
    // && !mLimelightInterface.tooClose();
  }
}
