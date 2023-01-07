// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous.AutonPaths;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.commands.Autonomous.AutoPaths;
import frc.robot.commands.Autonomous.FollowTrajectory;
import frc.robot.commands.Drive.ResetAll;
import frc.robot.commands.Drive.TeleopHeading;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.FloorIntake.LowerFloorIntake;
import frc.robot.commands.FloorIntake.RaiseFloorIntake;
import frc.robot.commands.FloorIntake.RunFloorIntake;
import frc.robot.commands.FloorIntake.RunFloorIntakeOut;
import frc.robot.commands.FloorIntake.RunOnlyFloorIntakeOut;
import frc.robot.commands.FloorIntake.ToggleFloorIntakeState;
import frc.robot.commands.Shooter.ShootHighAuto;
import frc.robot.commands.Shooter.ShooterRPMHigh;
import frc.robot.commands.Tower.RunTower;
import frc.robot.commands.Tower.RunTowerBack;
import frc.robot.commands.Tower.RunTower;
import frc.robot.commands.Vision.AlignToGoalAuton;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.utilities.DaisyMath;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class FiveBallHigh extends SequentialCommandGroup {
  /** Creates a new ThreeAllHigh. */
  public FiveBallHigh(Drivebase mDrivebase, FloorIntake floor, Tower tower, Shooter shooter) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      // Prepare for routine
      new ResetAll(mDrivebase).withTimeout(10.0),
      // Fire off the starting ball
      // new ShooterRPMSetPoint(shooter, 1600.0).alongWith(new RunTower(tower)).withTimeout(1.0).withInterrupt(()->tower.getBeamBreak()),
      // Follow the path to get in line with the balls
      new LowerFloorIntake(floor),
      new ParallelRaceGroup(
        new FollowTrajectory(mDrivebase, AutoPaths.fiveBall.get(0), "Part 1",
        true).withTimeout(10.0),
        new SequentialCommandGroup(
          new WaitCommand(0.5),
          new RunFloorIntake(floor, tower)
        )
      ).withTimeout(10.0),

      new ParallelDeadlineGroup(
        new AlignToGoalAuton(mDrivebase).withTimeout(2.5),
        new SequentialCommandGroup(
          new WaitCommand(2.75),
          new RunFloorIntakeOut(floor, tower).withTimeout(0.25)
        )
      ),
   
      // new AutoRunFloorIntakeOut(floor, tower).withTimeout(0.1),
      
      new ParallelDeadlineGroup(
        new ShootHighAuto(shooter, tower, true, () -> Constants.Shooter.DEFAULT_HIGH_RPM),
        new RunOnlyFloorIntakeOut(floor).withTimeout(0.25)
      ),


      new FollowTrajectory(mDrivebase, AutoPaths.fiveBall.get(1), "Part 2", false).withTimeout(10.0),

      new RunFloorIntake(floor, tower).withTimeout(0.5),

      new FollowTrajectory(mDrivebase, AutoPaths.fiveBall.get(2), "Part 3", false).withTimeout(10.0),

      new ShootHighAuto(shooter, tower, true, () -> Constants.Shooter.DEFAULT_HIGH_RPM),

      new ParallelDeadlineGroup(
        new FollowTrajectory(mDrivebase, AutoPaths.fiveBall.get(3), "Part 4", false).withTimeout(10.0),
        new RunFloorIntake(floor, tower)
      ),

      // new TurnToAngle(mDrivebase, 160.0),
      // new AlignToGoalAuton(mDrivebase),
      // new ShooterRPMHigh(shooter, tower, true, () -> Constants.Shooter.DEFAULT_HIGH_RPM),

      new RaiseFloorIntake(floor),

      // new FollowTrajectory(mDrivebase, AutoPaths.threeballhighpart4, "GoToStart"),

      new TeleopHeading(mDrivebase, DaisyMath.boundAngleNeg180to180Degrees(Constants.Auto.RIGHT_TARMAC_START_ANGLE + 187.5)).withTimeout(10.0),
      new WaitCommand(0.5),
      new TurnToAngle(mDrivebase, 0.0)//-69.0)

    );
  }
}
