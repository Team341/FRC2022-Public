// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
// import com.revrobotics.ColorSensorV3;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Robot;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Tower extends SubsystemBase {

  private static Tower instance = null;

  public static Tower getInstance() {
    if (instance == null) {
      instance = new Tower();
    }
    return instance;
  }

  /**
   * Types of ball states
   */
  public static enum BallState {
    Empty,
    Red,
    Blue
    //Unknown TODO
  }

  public BallState mBallState;

  // creates a new NEO motor for the sorter
  private CANSparkMax mTowerMotor;
  // private final ColorSensorV3 mColorSensor;

  private final DigitalInput mBeamBreakTop;
  private final DigitalInput mBeamBreakBottom;

  boolean forward = false;
  boolean backwards = false;

  public int ballsInRobot = 0;
  boolean bottomBeamBreakBroken = false;
  boolean topBeamBreakBroken = false;

  public BallState[] mQueue = new BallState[2];
  Solenoid mHardStopSolenoid;

  boolean mHardStopState;

  /** Creates a new Tower. */
  public Tower() {
    mTowerMotor = new CANSparkMax(Constants.Tower.TOWER_MOTOR_PORT, MotorType.kBrushless);
    // mColorSensor = new ColorSensorV3(I2C.Port.kOnboard);

    mTowerMotor.restoreFactoryDefaults();
    mTowerMotor.setSmartCurrentLimit(Constants.Tower.TOWER_CURRENT_LIMIT);
    mTowerMotor.setIdleMode(IdleMode.kBrake);
    mTowerMotor.setInverted(false);

    mBeamBreakTop = new DigitalInput(Constants.Tower.TOP_BEAM_BREAK_PORT);
    mBeamBreakBottom = new DigitalInput(Constants.Tower.BOTTOM_BEAM_BREAK_PORT);

    mHardStopSolenoid = new Solenoid(PneumaticsModuleType.CTREPCM,
        Constants.Tower.HARD_STOP_SOLENOID_PORT);
    mHardStopState = false;
    mQueue[0] = BallState.Blue;
    mQueue[1] = BallState.Empty;

  }

  /**
   * runs the sorter motor at speed defined in constants
   */
  public void run() {
    mTowerMotor.set(Constants.Tower.TOWER_FORWARDS_SPEED);
  }

  /**
   * runs the sorter motor backwards at speed defined in constants
   */
  public void runBackwards() {
    mTowerMotor.set(Constants.Tower.TOWER_BACKWARDS_SPEED);
  }

  /**
   * runs the sorter motor backwards at speed defined in constants
   */
  public void autoRunBackwards() {
    mTowerMotor.set(Constants.Tower.AUTO_TOWER_BACKWARDS_SPEED);
  }

  
  /**
   * sets the speed of the sorter motor
   * 
   * @param speed double [-1, 1]
   */
  public void setSpeed(double speed) {
    mTowerMotor.set(speed);
  }

  /**
   * @return RGB value of sensor reading
   */
  // public Color getColor() {
  //   return mColorSensor.getColor();
  // }

  /**
   * @return proximity to object (Higher value = closer max 2047)
   */
  // public int getProximity() {
  //   return mColorSensor.getProximity();
  // }

  /**
   * @return if ball leaves from top
   */
  public boolean getTopBeamBreak() {
    return !mBeamBreakTop.get();
  }

  /**
   * @return if the ball is detected
   */
  public boolean getBottomBeamBreak() {
    // return getProximity() > Constants.Tower.BALL_DETECTING_DISTANCE;
    return mBeamBreakBottom.get(); // TODO ! or not
  }

  /**
   * @return how many balls in robot
   */
  public int getBallsInRobotCount() {
    return ballsInRobot;
  }

  public void openHardStop() {
    if (!mHardStopState) {
      mHardStopSolenoid.set(true);
    }
    mHardStopState = true;
  }

  public void closeHardStop() {
    if (mHardStopState) {
      mHardStopState = false;
    }
    mHardStopSolenoid.set(false);
  }

  public boolean getHardStopState() {
    mHardStopState = mHardStopSolenoid.get();
    return mHardStopState;
  }

  /**
   * @return queue of balls in robot (top to bottom)
   */
  public BallState[] getBallStates() {
    return mQueue;
  }

  /**
   * @return if ball being read is blue
   */
  // private boolean getIsBallBlue() {
  //   Color c = getColor();
  //   return c.blue > c.red;
  // }

  // /**
  //  * @return if ball being read is red
  //  */
  // private boolean getIsBallRed() {
  //   Color c = getColor();
  //   return c.blue < c.red;
  // }

  /**
   * @return queue as string (top to bottom)
   */
  private String getCurrentQueue() {
    String tString = "";
    for (BallState state : mQueue) {
      if (state == BallState.Empty) {
        tString += "Empty ";
      } else if (state == BallState.Blue) {
        tString += "Blue ";
      } else if (state == BallState.Red) {
        tString += "Red ";
      }
    }
    return tString;
  }

  private void logToDashboardInDetail() {
    // SmartDashboard.putBoolean("Tower/Hard Stop State", mHardStopState);
    // SmartDashboard.putNumber("Tower/Proximity", getProximity());
    // SmartDashboard.putNumber("Tower/Balls in Robot", ballsInRobot);
    // SmartDashboard.putBoolean("Tower/Is Ball Blue", getIsBallBlue());

  }

  private void logToDashboard() {
    // SmartDashboard.putString("Tower/Ball Queue", getCurrentQueue());
    // SmartDashboard.putBoolean("Tower/Color Sensor Ball Detected",
    // colorSensorBallDetected());

    SmartDashboard.putBoolean("Tower/Beam Break Top Ball Detected", getTopBeamBreak());
    SmartDashboard.putBoolean("Tower/Beam Break Bottom Ball Detected", getBottomBeamBreak());

    // SmartDashboard.putNumber("Tower/Red Color Value", getColor().red);
    // SmartDashboard.putNumber("Tower/Green Color Value", getColor().green);
    // SmartDashboard.putNumber("Tower/Blue Color Value", getColor().blue);
  }

  /**
   * @return first ball (bottom)
   */
  public BallState getBottomBallState() {
    return mQueue[1];
  }

  /**
   * @return second ball (top)
   */
  public BallState getTopBallState() {
    return mQueue[0];
  }

  /**
   * @return enum of ball states
   */
  public BallState getBallStateEnum() {
    return mBallState;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    // SmartDashboard.put
    // if (getProximity() > Constants.Tower.BALL_DETECTING_DISTANCE) {
    if (getBottomBeamBreak()) {
      if (FloorIntake.getInstance().getMotorGoingBackwards() && !backwards) {
        backwards = true;
        if (mQueue[0] == BallState.Empty || mQueue[1] == BallState.Empty) {
          mQueue[0] = BallState.Empty;
          mQueue[1] = BallState.Empty;
        }
        else {
          mQueue[1] = mQueue[0];
          mQueue[0] = BallState.Empty;
        }
      }
      else {
        backwards = false;
      }
      if (!bottomBeamBreakBroken) {
        // // needs to check for both ball colors since ball is in proximity before correct
        // // color is detected
        if (!FloorIntake.getInstance().getMotorGoingBackwards() && !forward) {
          forward = true;
        //   if (mQueue[0] == BallState.Empty || mQueue[1] == BallState.Empty) {
        //     mQueue[0] = BallState.Empty;
        //     mQueue[1] = BallState.Empty;
        //   }
        //   else {
        //     mQueue[1] = mQueue[0];
        //     mQueue[0] = BallState.Empty;
        //   }
          // SmartDashboard.putBoolean("Going Backwords", true);
        // } else {
            if (getBottomBallState() == BallState.Empty && getTopBallState() == BallState.Empty) { // puts blue ball on
                                                                                                   // top of queue if
                                                                                                   // completely empty
              mQueue[0] = BallState.Blue;
            } else {
              mQueue[1] = mQueue[0];
              mQueue[0] = BallState.Blue;

            }

        }
        else {
          forward = false;
        }
      }
      bottomBeamBreakBroken = true;
    } else {
      forward = false;
      backwards = false;
      bottomBeamBreakBroken = false;
    }
    // if (SmartDashboard.getBoolean("Tower/Beam Break State", false)) { //TODO
    // replace with real beam break readings
    if (getTopBeamBreak()) {
      // if beam breaks ball leaves from shooter
      if (!topBeamBreakBroken) {
        ballsInRobot--;

        mQueue[0] = mQueue[1];
        mQueue[1] = BallState.Empty;

      }
      topBeamBreakBroken = true;
    } else {
      topBeamBreakBroken = false;
    }

    // if (Robot.detailedLogging) {
    //   logToDashboardInDetail();
    // }
    if (Robot.logging) {
      logToDashboard();
    }
    // System.out.println("Tower");
  }
}
