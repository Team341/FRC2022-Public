/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.FloorIntake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Tower;

public class AutoRunFloorIntakeOut extends CommandBase {
  private FloorIntake mFloorIntake;
  private Tower mTower;

  private boolean mIntakeState;
  private boolean mLoweredIntake;
  
  /**
   * Reverses intake
   * @param floor
   */
  public AutoRunFloorIntakeOut(FloorIntake floor, Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    mFloorIntake = floor;
    mTower = tower;
    // addRequirements(mFloorIntake, mTower);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // mIntakeState = mFloorIntake.getFloorIntakeState();
    mFloorIntake.deployIntake();
    mFloorIntake.autoReverseIntake();
    mTower.autoRunBackwards();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mFloorIntake.autoReverseIntake();
    mTower.autoRunBackwards();
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFloorIntake.setIntakeSpeed(0.0);
    mTower.setSpeed(0.0);

    // mFloorIntake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}