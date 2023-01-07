// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber.AutoClimb;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Constants;
import frc.robot.RobotContainer.ClimbSelector;
import frc.robot.commands.Climber.SetBothHookPosition;
import frc.robot.commands.Climber.SetClimbStage;
import frc.robot.commands.Climber.SetClimbingStatus;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.commands.Drive.DriveBackUntilLimitSwitchPressed;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.Drivebase;

public class LockOnMidBar extends SequentialCommandGroup {
  public LockOnMidBar(ClimberHooks climberHooks, Climber climber, Drivebase drivebase) {
    addCommands(
      new SetClimbingStatus(() -> false),
      new SetBothHookPosition(climberHooks, () -> Constants.Climber.LONG_SIDE_ACCEPTING_POSITION, () -> Constants.Climber.SHORT_SIDE_ACCEPTING_POSITION),
      new DriveBackUntilLimitSwitchPressed(drivebase, climberHooks), // hooks long side onto mid bar 
      new SetLongSidePosition(climberHooks, 0.0),
      new SetClimbingStatus(() -> true),
      new SetClimbStage(climber, ClimbSelector.THREE)
    );
  }
}
