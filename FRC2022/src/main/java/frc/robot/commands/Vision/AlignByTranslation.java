// CopyLeft (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.utilities.DaisyMath;

public class AlignByTranslation extends CommandBase {

  private LimelightInterface mPhotonVisionInterface;
  private Drivebase mDrivebase;
  private DoubleSupplier mLeftY;
  private DoubleSupplier mLeftX;
  private double mLimelighttx;
  private double targetAngle;
  
  /**
   * Aligns through translation
   * @param drivebase
   * @param leftY left joystick y
   * @param leftX left joystick x
   */
  public AlignByTranslation(Drivebase drivebase, DoubleSupplier leftY, DoubleSupplier leftX) {
    // Use addRequirements() here to declare subsystem dependencies.
    mPhotonVisionInterface = LimelightInterface.getInstance();
    mDrivebase = drivebase;
    mLeftY = leftY;
    mLeftX = leftX;
    targetAngle = 0.0;
    mLimelighttx = mPhotonVisionInterface.tx;

    addRequirements(mDrivebase);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mPhotonVisionInterface.setLimeLightLED(3);

    targetAngle = DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() - mLimelighttx);
    mDrivebase.getThetaController().setSetpoint(targetAngle);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double xSpeed = Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND * mLeftX.getAsDouble();
    double ySpeed = Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND * mLeftY.getAsDouble();

    // double rot = Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND *
    //     mDrivebase.getThetaController().calculate(mDrivebase.getAngle().getDegrees(), 
    //     mDrivebase.getAngle().getDegrees() - mLimelighttx);

    mDrivebase.drive(new ChassisSpeeds(xSpeed, ySpeed, 0.0));
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {

    return Math.abs(
      DaisyMath.boundAngleNeg180to180Degrees(mDrivebase.getAngle().getDegrees() - mLimelighttx)
      ) < Constants.Drivebase.VISION_ANGLE_TOLERANCE;
  }
}

