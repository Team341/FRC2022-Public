// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Climber;

// import java.util.function.IntSupplier;

// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
// import frc.robot.Robot;
// import frc.robot.RobotContainer;
// import frc.robot.commands.Climber.AutoClimb.DriveToMidBar;
// import frc.robot.commands.Climber.AutoClimb.LockOnMidBar;
// import frc.robot.commands.Climber.AutoClimb.LockOnToTraversal;
// import frc.robot.commands.Climber.AutoClimb.RotateToHighBar;
// import frc.robot.commands.Climber.AutoClimb.RotateToTraversal;
// import frc.robot.subsystems.Climber;
// import frc.robot.subsystems.ClimberHooks;
// import frc.robot.subsystems.Drivebase;

// // NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// // information, see:
// // https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
// public class AdvanceClimb extends SequentialCommandGroup {
//   /** Creates a new AdvanceClimb. */
//   public AdvanceClimb(Climber climber, ClimberHooks climberHooks, Drivebase drivebase, IntSupplier integer) {
//     // Add your commands in the addCommands() call, e.g.
//     // addCommands(new FooCommand(), new BarCommand());

//     SmartDashboard.putNumber("Testing/Before Climb Stage", climber.climbStage);
//     climber.climbStage += 1;
//     SmartDashboard.putNumber("Testing/After Climb Stage", climber.climbStage);
    
//     switch (integer.getAsInt()) {
//       case 1:
//         addCommands(new DriveToMidBar(climber, climberHooks, drivebase));
//         break;
//       case 2:
//         addCommands(new LockOnMidBar(climberHooks));
//         break;
//       case 3:
//         addCommands(new RotateToHighBar(climber, climberHooks));
//         break;
//       case 4:
//         addCommands(new RotateToTraversal(climber, climberHooks));
//         break;
//       case 5:
//         addCommands(new LockOnToTraversal(climber, climberHooks));
//         break;
//     }
//   }
// }
