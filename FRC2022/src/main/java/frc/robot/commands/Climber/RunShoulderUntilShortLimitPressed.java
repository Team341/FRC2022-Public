// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

public class RunShoulderUntilShortLimitPressed extends CommandBase {

  Climber mClimber;
  ClimberHooks mClimberHooks;

  /** Creates a new RunShoulderUntilShortLimitPressed. */
  public RunShoulderUntilShortLimitPressed(Climber climber, ClimberHooks climberHooks) {
    addRequirements(mClimber);
    //addRequirements(mClimberHooks);
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // System.out.println("Starting to rotate shoulder");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // System.out.println("Rotating Shoulder");
    mClimber.setSpeed(Constants.Climber.SHOULER_SPEED);
        // if (mForwards) {
    // I consider this to be adequately addressed by being willing to make shoulder speed be negative
    // It always goes the same way
    // } 
    // else {
      // mClimber.setSpeed(-1.0 * Constants.Climber.SHOULER_SPEED);
    // }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mClimber.setSpeed(0.0);
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    // Do we need/want the or here? Something to think about. 
    return mClimberHooks.getLeftShortLimitSwitch() || mClimberHooks.getRightShortLimitSwitch();
  }
}