// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot.commands.Tower;

// import edu.wpi.first.wpilibj2.command.CommandBase;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.Tower;

// public class EmptyTower extends CommandBase {

//   private Tower mTower;
//   private Shooter mShooter;
  
//   private boolean mIsShooting;
//   private Integer emptyCount;
//   private int countSinceShot;
//   public EmptyTower(Tower tower, Shooter shooter) {
//     // Use addRequirements() here to declare subsystem dependencies.
//     mTower = tower;
//     mShooter = shooter;
    
//     mIsShooting = false;
//     addRequirements(mTower);
//     emptyCount = 0;
//     countSinceShot = 0;
//   }

//   // Called when the command is initially scheduled.
//   @Override
//   public void initialize() {
//     mTower.openHardStop();
//   }

//   // Called every time the scheduler runs while the command is scheduled.
//   @Override
//   public void execute() {
//     if (mShooter.getOnTargetCount() > 5 && mShooter.getOnRPMTarget()) {
//       mTower.run();
//     }

    
//   }

//   // Called once the command ends or is interrupted.
//   @Override
//   public void end(boolean interrupted) {
//     mTower.setSpeed(0.0);
//   }

//   // Returns true when the command should end.
//   @Override
//   public boolean isFinished() {
//     // We are done when no beam breaks are triggered
//     return mTower.getBallsInRobotCount() == 0;
//   }
// }
