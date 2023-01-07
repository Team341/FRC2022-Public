// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Tower;

// import java.util.function.DoubleSupplier;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Tower;

// public class RunTowerByJoystick extends CommandBase {
//   private Tower mTower;
//   private DoubleSupplier mJoystick;
//   /** Creates a new RunTowerByJoystick. */
//   public RunTowerByJoystick(Tower tower, DoubleSupplier joystick) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     mTower = tower;
//     mJoystick = joystick;

//     addRequirements(mTower);
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     mTower.closeHardStop();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     mTower.setSpeed(mJoystick.getAsDouble());
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {}

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     return false;
//   }
// }
