// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.AutoClimb;

import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.WaitUntilCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.commands.Climber.RunShoulderUntilInterrupted;
import frc.robot.commands.Climber.RunShoulderUntilShortLimitPressed;
import frc.robot.commands.Climber.SetClimbStage;
import frc.robot.commands.Climber.SetClimbingStatus;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class RotateToHighBar extends SequentialCommandGroup {
  /** Creates a new RotateToHighBar. */
  public RotateToHighBar(Climber climber, ClimberHooks climberHooks) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    // System.out.println("climber is " + climber.getName());//-31.0
    addCommands(
      new SetClimbingStatus(() -> false),
      new SetLongSidePosition(climberHooks, Constants.Climber.LEFT_LONG_CLOSED_POSITION),
      // Rotate to high bar
      new SetShortSidePosition(climberHooks, Constants.Climber.SHORT_SIDE_ACCEPTING_POSITION),
      // new RunShoulderUntilShortLimitPressed(climber, climberHooks), // aligns to high bar 
      // new PrintCommand("Running second command-------------------------------------------------------------------"),
      new RunShoulderUntilInterrupted(climber, ()->climberHooks.getShortSideEngaged()),
      new SetShortSidePosition(climberHooks, 0.0), // brings hook into lock on position
      new SetClimbStage(climber, ClimbSelector.FOUR),
      // new RunCommand( ()->climber.setSpeed(Constants.Climber.SHOULER_SPEED)).raceWith(new WaitUntilCommand(()->climberHooks.getShortSideEngaged()))
      new SetClimbingStatus(() -> true));
  }
}
