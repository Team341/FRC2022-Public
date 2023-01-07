// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Climber;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.WaitForDriverAcceptance;
import frc.robot.commands.Drive.DriveBackUntilLimitSwitchPressed;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.Drivebase;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ObseleteAutomatedClimb extends SequentialCommandGroup {

  /** Creates a new AutomatedClimb. */
  public ObseleteAutomatedClimb(Climber climber, ClimberHooks climberHooks, Drivebase drivebase, XboxController controller) {
    addCommands(

      // Align to mid bar
      new ParallelCommandGroup(
        new SetLongSidePosition(climberHooks, 120.0), // brings hook into position to latch
        new ShoulderSetPoint(climber, 0.0) // gets climber into position 
      ),
      new DriveBackUntilLimitSwitchPressed(drivebase, climberHooks), // hooks long side onto mid bar 
      new WaitForDriverAcceptance(controller), 

      // Grab mid bar
      new ParallelCommandGroup(
        new SetLongSidePosition(climberHooks, 130.0), // brings hook into lock on position
        new SetShortSidePosition(climberHooks, 120.0) // brings hook into acceptance position  
      ),

      // Rotate to high bar
      new RunShoulderUntilShortLimitPressed(climber, climberHooks), // aligns to high bar 
      new SetShortSidePosition(climberHooks, 130.0), // brings hook into lock on position
      new WaitForDriverAcceptance(controller),     
      

      // Rotate to traversal
      new SetLongSidePosition(climberHooks, 0.0), // unhooks from mid bar
      new ParallelCommandGroup(
        new RunShoulderUntilLongLimitPressed(climber, climberHooks), //aligns to traversal
        new SequentialCommandGroup(
          new WaitForLongSideRelease(climberHooks),
          new SetLongSidePosition(climberHooks, 120.0) // brings hook into position to latch
        )
      ),
      new SetLongSidePosition(climberHooks, 130.0), //hook onto traversal

        
      // Lock on to traversal
      new WaitForDriverAcceptance(controller), 
      new SetShortSidePosition(climberHooks, 0.0), //unhook from high bar

      // Hang directly below traversal
      new ShoulderSetPoint(climber, 0.0)
    );   
  }
}
