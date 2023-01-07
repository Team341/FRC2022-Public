// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.AutoClimb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.commands.Climber.SetClimbStage;
import frc.robot.commands.Climber.SetClimbingStatus;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetRainbow;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.commands.Climber.ShoulderSetPoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class LockOnToTraversal extends SequentialCommandGroup {
  /** Creates a new LockOnToTraversal. */
  public LockOnToTraversal(Climber climber, ClimberHooks climberHooks) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClimbingStatus(() -> false),
      new SetLongSidePosition(climberHooks, 0.0), //hook onto traversal
      new ShoulderSetPoint(climber, -200.0),
      new WaitCommand(0.25),
      new SetShortSidePosition(climberHooks, Constants.Climber.SHORT_SIDE_RELEASE_POSITION - 3.0), //, //unhook from high bar
      new SetClimbStage(climber, ClimbSelector.SIX),

      // Hang directly below traversal
      new ShoulderSetPoint(climber, -270.0),
      // new SetClimbingStatus(() -> true));
      new SetRainbow(() -> true));
  }
}
