// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.ClimberHooks;

public class SetBothHookPosition extends CommandBase {
  ClimberHooks mClimberHooks;
  DoubleSupplier mLongDegrees;
  DoubleSupplier mShortDegrees;
  // double mAlphaPosition;
  /** Creates a new SetBothHookPosition. */
  public SetBothHookPosition(ClimberHooks climberHooks, DoubleSupplier longDegrees, DoubleSupplier shortDegrees) {
    mClimberHooks = climberHooks;
    mLongDegrees = longDegrees;
    mShortDegrees = shortDegrees;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimberHooks);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Initializing Hook Gains");
    // Constants.Climber.HOOK_MOTOR_PID_GAINS.updateFromDashboard();
    // System.out.println("Updated from smart dashboard");
    // mClimber.setHookPIDs(Constants.Climber.HOOK_MOTOR_PID_GAINS);
    // System.out.println("Updated in subsystem");
    // SmartDashboard.putNumber("Updated Hook kP", Constants.Climber.HOOK_MOTOR_PID_GAINS.kP);
    //mAlphaPosition = mClimber. 
    // mAlpha = SmartDashboard.getNumber("Hook/Alpha", Constants.Climber.HOOK_ALPHA);
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double longSide = mLongDegrees.getAsDouble();
    double shortSide = mShortDegrees.getAsDouble();
    mClimberHooks.setLongSidePosition(longSide, mClimberHooks.getHookAlpha());

    mClimberHooks.setShortSidePosition(shortSide, mClimberHooks.getHookAlpha());
    // System.out.println("Running Hook Position Control");
    // mAlphaPosition = mAlpha * mSetPoint + (1 - mAlpha) * mAlphaPosition;
    // mClimber.SetBothHookPosition(mAlphaPosition);
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
    // return Math.abs(mClimber.getShoulderPosition() - mSetPoint) < Constants.Climber.POSITION_TOLERANCE;
    return mClimberHooks.longSideHooksInPosition() && mClimberHooks.shortSideHooksInPosition();
  }
}
