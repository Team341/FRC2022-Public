// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Shooter;

import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.Constants;
// import frc.robot.commands.Tower.EmptyTower;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class ShootAllBalls extends ParallelCommandGroup {
  
  /**
   * All balls are shot
   * @param shooter
   * @param tower
   * @param enableLimelight whether limeight is enabled
   */
  public ShootAllBalls(Shooter shooter, Tower tower, boolean enableLimelight) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
        new ShooterRPMLow(shooter, tower, enableLimelight, () -> Constants.Shooter.DEFAULT_RPM_SET_POINT)//,
        // new EmptyTower(tower, shooter)
    );
  }
}
