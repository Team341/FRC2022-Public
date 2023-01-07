// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.AutoClimb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.commands.Climber.RunClimber;
import frc.robot.commands.Climber.RunShoulderUntilInterrupted;
import frc.robot.commands.Climber.RunShoulderUntilLongLimitPressed;
import frc.robot.commands.Climber.SetClimbStage;
import frc.robot.commands.Climber.SetClimbingStatus;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetRainbow;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.commands.Climber.ShoulderSetPoint;
import frc.robot.commands.Climber.WaitForLongSideRelease;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToTraversal extends SequentialCommandGroup {
  /** Creates a new RotateToTraversal. */
  public RotateToTraversal(Climber climber, ClimberHooks climberHooks) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClimbingStatus(() -> false),
      new SetLongSidePosition(climberHooks, 0.0),
      new SetShortSidePosition(climberHooks, 0.0), // brings hook into lock on position
      new ShoulderSetPoint(climber, -15.0), // Bring our shoulder to vertical before releasing
      new WaitCommand(0.25),
      new SetLongSidePosition(climberHooks, Constants.Climber.LONG_SIDE_RELEASE_POSITION - 5.0), // unhooks from mid bar
      // new ShoulderSetPoint(climber, -90.0),
      // new SetClimbStage(climber, ClimbSelector.FIVE),
      // new SetRainbow(() -> true));
      new RunClimber(climber, () -> Constants.Climber.SHOULER_SPEED).withTimeout(0.5),
      new ParallelCommandGroup(
        new RunShoulderUntilInterrupted(climber, ()->climberHooks.getLongSideEngaged()), //aligns to traversal
        new SequentialCommandGroup(
          new WaitForLongSideRelease(climberHooks),
          new SetLongSidePosition(climberHooks, Constants.Climber.LONG_SIDE_ACCEPTING_POSITION) // brings hook into position to latch
        )
      ),
      
      new SetLongSidePosition(climberHooks, 0.0), //hook onto traversal
      // new ShoulderSetPoint(climber, degree),
      new SetClimbStage(climber, ClimbSelector.FIVE),
      new SetClimbingStatus(() -> true));
      // new SetClimbStage(climber, ClimbSelector.FIVE),
      // new SetRainbow(() -> true));
  }
}
