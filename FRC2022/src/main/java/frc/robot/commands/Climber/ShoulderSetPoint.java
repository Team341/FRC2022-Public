// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;

public class ShoulderSetPoint extends CommandBase {

  private Climber mClimber;
  private double mSetPoint;
  private double mSetPointDelta;

  /** Creates a new ShoulderSetPoint. */
  public ShoulderSetPoint(Climber climber, double degree) {
    mClimber = climber;
    mSetPoint = degree;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Constants.Climber.SHOULDER_MOTOR_PID_GAINS.updateFromDashboard();
    mClimber.setShoulderGains();

    mSetPointDelta = mSetPoint- mClimber.getShoulderPosition();
    mClimber.setShoulderPosition(mSetPoint);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimber.setShoulderPosition(mSetPoint);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Math.abs(mClimber.getShoulderPosition() - mSetPoint) <= Constants.Climber.SHOULDER_ANGLE_TOLERANCE;
    // return true;
  }
}
