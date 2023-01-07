// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.Climber;

public class RunClimber extends CommandBase {

  private Climber mClimber;
  private DoubleSupplier mSpeed;
  /** Creates a new RunClimber. */
  public RunClimber(Climber climber, DoubleSupplier speed) {

    mClimber = climber;
    mSpeed = speed;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(mClimber);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mClimber.setSpeed(mSpeed.getAsDouble());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
