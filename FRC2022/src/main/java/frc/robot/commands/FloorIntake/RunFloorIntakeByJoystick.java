/*----------------------------------------------------------------------------*/
/* Copyright (c) 2019 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.commands.FloorIntake;

import java.util.function.DoubleSupplier;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.Tower;;

public class RunFloorIntakeByJoystick extends CommandBase {
  /**
   * Creates a new RunFloorIntakeByJoystick.
   */
  private FloorIntake mFloorIntake;
  private DoubleSupplier mSpeed;
  int mBallCount;
  Tower mTower;

  // private boolean mIntakeState;
  // private boolean mLoweredIntake;

  /**
   * Intake controlled by a joystick
   * 
   * @param floorIntake
   * @param speed
   * @param tower
   */
  public RunFloorIntakeByJoystick(FloorIntake floorIntake, DoubleSupplier speed, Tower tower) {
    // Use addRequirements() here to declare subsystem dependencies.
    mTower = tower;
    mSpeed = speed;
    mFloorIntake = floorIntake;

    addRequirements(mFloorIntake);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    mBallCount = mTower.getBallsInRobotCount();
    mFloorIntake.deployIntake();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    mBallCount = mTower.getBallsInRobotCount();
    if (mBallCount < Constants.FloorIntake.MAX_BALL_COUNT) { // does not intake if tower is full

      mFloorIntake.setIntakeSpeed(mSpeed.getAsDouble());
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    mFloorIntake.setIntakeSpeed(0.0);
    mFloorIntake.retractIntake();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}