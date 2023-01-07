// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.AutoClimb;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.commands.Climber.SetClimbStage;
import frc.robot.commands.Climber.SetClimbingStatus;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.commands.Climber.ShoulderSetPoint;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class PrepareForMidBar extends SequentialCommandGroup {
  /** Creates a new PrepareForMidBar. */
  public PrepareForMidBar(ClimberHooks climberHooks, Climber climber) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new SetClimbingStatus(() -> false),
      // Align to mid bar
      new ParallelCommandGroup(
        new SetLongSidePosition(climberHooks, Constants.Climber.LONG_SIDE_ACCEPTING_POSITION), // brings hook into position to latch
        new SetShortSidePosition(climberHooks, Constants.Climber.SHORT_SIDE_ACCEPTING_POSITION), 
        new ShoulderSetPoint(climber, 90.0) // gets climber into position 
      ),
      new SetClimbingStatus(() -> true),
      new SetClimbStage(climber, ClimbSelector.TWO)
    );
  }
}
