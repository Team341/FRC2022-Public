// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Vision;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.Drivebase;
import frc.robot.Constants;
import frc.robot.commands.Shooter.ShooterRPMLow;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.tracking.LimelightInterface;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class AlignAndShoot extends SequentialCommandGroup {
  
  /**
   * Aligns to goal and shoots
   * @param drivebase
   * @param leftY left joystick y
   * @param leftX left joystick x 
   * @param shooter
   * @param tower
   */
  public AlignAndShoot(Drivebase drivebase, DoubleSupplier leftY, DoubleSupplier leftX, DoubleSupplier turn, Shooter shooter, Tower tower) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new AlignToGoal(drivebase, leftY, leftX, turn),
      new ShooterRPMLow(shooter, tower, true, () -> Constants.Shooter.DEFAULT_RPM_SET_POINT)
    );
  }
}
