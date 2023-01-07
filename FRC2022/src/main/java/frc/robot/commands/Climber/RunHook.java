// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.ClimberHooks;

public class RunHook extends CommandBase {

  private ClimberHooks mClimberHooks;
  private DoubleSupplier mShortSideSpeed;
  private DoubleSupplier mLongSideSpeed;


  /** Creates a new RunHook. */
  public RunHook(ClimberHooks climber, DoubleSupplier shortSideSpeed, DoubleSupplier longSideSpeed) {
    mClimberHooks = climber;
    // Use addRequirements() here to declare subsystem dependencies.
    //addRequirements(mClimber);

    mShortSideSpeed = shortSideSpeed;
    mLongSideSpeed = longSideSpeed;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    double longSide = mLongSideSpeed.getAsDouble();
    double shortSide = mShortSideSpeed.getAsDouble();
    mClimberHooks.setHookMotorLeftLongSideSpeed(longSide);
    mClimberHooks.setHookMotorRightLongSideSpeed(longSide);
    mClimberHooks.setHookMotorLeftShortSideSpeed(shortSide);
    mClimberHooks.setHookMotorRightShortSideSpeed(shortSide);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
