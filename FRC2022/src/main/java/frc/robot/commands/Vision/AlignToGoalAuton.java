// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.utilities.DaisyMath;

// 1640 do this here:https://github.com/FRC1640/2021-Code/blob/main/src/main/java/frc/robot/autonomous/commands/AlignAuton.java

public class AlignToGoalAuton extends CommandBase {

  private LimelightInterface mPhotonVisionInterface;
  private Drivebase mDrivebase;
  
  // // PID Values for turn
  // private double kP = Constants.Vision.VISION_PID_GAINS.kP;
  // private double kI =  Constants.Vision.VISION_PID_GAINS.kI;
  // private double kD =  Constants.Vision.VISION_PID_GAINS.kD;
  private double kF = Constants.Drivebase.VISION_KF; /// 2.0;

  private double targetAngle;
  private double maxOutput;
  double filteredTx;

  int count;


  /**
   * Original align to goal
   * 
   * @param drivebase
   */
  public AlignToGoalAuton(Drivebase drivebase) {
    // Use addRequirements() here to declare subsystem dependencies.
    mPhotonVisionInterface = LimelightInterface.getInstance();
    mDrivebase = drivebase;
    targetAngle = 0.0;
    maxOutput = 0.35;

    // SmartDashboard.putNumber("Vision/kP", kP);
    // SmartDashboard.putNumber("Vision/kI", kI);
    // SmartDashboard.putNumber("Vision/kD", kD);
    SmartDashboard.putNumber("Vision/Max Turn Output", maxOutput);
    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    LED.getInstance().setAutoLED();
    SmartDashboard.putNumber("Vision/Max Turn Output", maxOutput);
    mPhotonVisionInterface.setLimeLightLED(3);

    count = 0;

    filteredTx = mPhotonVisionInterface.tx;

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
    
    

    filteredTx = (Constants.Vision.TX_FILTER * mPhotonVisionInterface.tx)
        + ((1 - Constants.Vision.TX_FILTER) * filteredTx);


    double rot = mPhotonVisionInterface.mLimelightController.calculate(-1.0 * filteredTx);

    double error = DaisyMath.boundAngleNeg180to180Degrees(filteredTx) * kF;

    rot += error;

    // double error = DaisyMath.boundAngleNeg180to180Degrees(filteredTx);
    // double rot = (kF * error) + (kP * error);// DaisyMath.boundAngleNeg180to180Degrees(mLimelightInterface.mLimelightController.getSetpoint()
                                                                                                    // -
                                                                                                    // mDrivebase.getAngle().getDegrees()));

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
    if (!mPhotonVisionInterface.hasTarget()) {
      rot = -1.0;
    }

    if (Math.abs(filteredTx) < Constants.Drivebase.VISION_ANGLE_TOLERANCE) {
      count++;
    }

    mDrivebase.drive(
        ChassisSpeeds.fromFieldRelativeSpeeds(0.0, 0.0, /* rot + */ rot, mDrivebase.getAngle()));

    SmartDashboard.putNumber("Vision/Filtered TX", filteredTx);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    LED.getInstance().setTeleOpLED();
    mDrivebase.drive(new ChassisSpeeds());
    count = 0;
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
    return count >= 5;
  }
}
