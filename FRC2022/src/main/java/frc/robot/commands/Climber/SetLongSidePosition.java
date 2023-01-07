// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

public class SetLongSidePosition extends CommandBase {
  ClimberHooks mClimberHooks;
  double mSetPoint;
  // double mAlphaPosition;
  /** Creates a new SetLongSidePosition. */
  public SetLongSidePosition(ClimberHooks climberHooks, double degrees) {
    mClimberHooks = climberHooks;
    mSetPoint = degrees;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(mClimberHooks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climber.getInstance().mClimbing = true;
    // System.out.println("Initializing Hook Gains");
    // Constants.Climber.HOOK_MOTOR_PID_GAINS.updateFromDashboard();
    // System.out.println("Updated from smart dashboard");
    // mClimberHooks.setHookPIDs(Constants.Climber.HOOK_MOTOR_PID_GAINS);
    // System.out.println("Updated in subsystem");
    // SmartDashboard.putNumber("Updated Hook kP", Constants.Climber.HOOK_MOTOR_PID_GAINS.kP);
    //mAlphaPosition = mClimberHooks. 
    // mAlpha = SmartDashboard.getNumber("Hook/Alpha", Constants.Climber.HOOK_ALPHA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimberHooks.setLongSidePosition(mSetPoint, mClimberHooks.getHookAlpha());
    // System.out.println("Running Hook Position Control");
    // mAlphaPosition = mAlpha * mSetPoint + (1 - mAlpha) * mAlphaPosition;
    // mClimberHooks.setLongSidePosition(mAlphaPosition);
    // SmartDashboard.putNumber("Hook/Alpha Position", mAlphaPosition);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    // System.out.println("Done Running Hook Position");
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // return Math.abs(mClimberHooks.getShoulderPosition() - mSetPoint) < Constants.Climber.POSITION_TOLERANCE;
    return mClimberHooks.longSideHooksInPosition();
  }
}
