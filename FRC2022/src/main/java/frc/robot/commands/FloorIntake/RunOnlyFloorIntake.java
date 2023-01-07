// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.FloorIntake;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntake;

public class RunOnlyFloorIntake extends CommandBase {
  private FloorIntake mFloorIntake;
  /** Creates a new RunOnlyFloorIntake. */
  public RunOnlyFloorIntake(FloorIntake floorIntake) {
    // Use addRequirements() here to declare subsystem dependencies.
    mFloorIntake = floorIntake;
    addRequirements(mFloorIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFloorIntake.runIntake();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFloorIntake.setIntakeSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
