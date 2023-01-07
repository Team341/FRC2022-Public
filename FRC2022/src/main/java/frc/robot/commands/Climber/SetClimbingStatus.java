// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class SetClimbingStatus extends CommandBase {
  BooleanSupplier mStatus; 
  /** Creates a new SetClimbingStatus. */
  public SetClimbingStatus(BooleanSupplier status) {
    // Use addRequirements() here to declare subsystem dependencies.
    mStatus = status;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Climber.climbing = mStatus.getAsBoolean();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
