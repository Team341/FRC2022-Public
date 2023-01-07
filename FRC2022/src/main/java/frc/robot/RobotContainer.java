// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.FileNotFoundException;
import java.util.Map;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SelectCommand;
import edu.wpi.first.wpilibj2.command.button.Button;
import frc.robot.commands.SpitOutBadBalls;
import frc.robot.commands.Autonomous.AutoPaths;
import frc.robot.commands.Autonomous.FollowTrajectory;
import frc.robot.commands.Autonomous.AutonPaths.DriveAndTurn;
import frc.robot.commands.Autonomous.AutonPaths.ThreeBallHighTest;
import frc.robot.commands.Autonomous.AutonPaths.TwoBallHigh;
import frc.robot.commands.Climber.GoToClosedPosition;
// import frc.robot.commands.Climber.AdvanceClimb;
import frc.robot.commands.Climber.RequireClimberAndHooks;
import frc.robot.commands.Climber.ResetArmEncoders;
import frc.robot.commands.Climber.ResetHookReferencePoint;
import frc.robot.commands.Climber.RunClimber;
import frc.robot.commands.Climber.RunHook;
import frc.robot.commands.Climber.SetLongSidePosition;
import frc.robot.commands.Climber.SetShortSidePosition;
import frc.robot.commands.Climber.ShoulderSetPoint;
import frc.robot.commands.Climber.AutoClimb.DriveToMidBar;
import frc.robot.commands.Climber.AutoClimb.LockOnMidBar;
import frc.robot.commands.Climber.AutoClimb.LockOnToTraversal;
import frc.robot.commands.Climber.AutoClimb.PrepareForMidBar;
import frc.robot.commands.Climber.AutoClimb.RotateToHighBar;
import frc.robot.commands.Climber.AutoClimb.RotateToTraversal;
import frc.robot.commands.Drive.ChaseTagCommand;
import frc.robot.commands.Drive.DiamondFormation;
import frc.robot.commands.Drive.DriveFalconOnly;
import frc.robot.commands.Drive.FieldOrientedDrive;
import frc.robot.commands.Drive.ReducedSpeed;
import frc.robot.commands.Drive.InvertTurnMotors;
import frc.robot.commands.Drive.OffsetRobotCenter;
import frc.robot.commands.Drive.ResetAll;
import frc.robot.commands.Drive.TurnToAngle;
import frc.robot.commands.FloorIntake.RunFloorIntake;
import frc.robot.commands.FloorIntake.RunFloorIntakeOut;
import frc.robot.commands.FloorIntake.ToggleFloorIntakeState;
import frc.robot.commands.LED.RunLED;
import frc.robot.commands.Shooter.ShootHighAuto;
import frc.robot.commands.Shooter.ShootRPMSetPoint;
import frc.robot.commands.Shooter.ShooterRPMHigh;
import frc.robot.commands.Shooter.ShooterRPMLow;
import frc.robot.commands.Shooter.ShooterRPMSetPoint;
import frc.robot.commands.Tower.RunTower;
import frc.robot.commands.Vision.AlignToGoal;
import frc.robot.commands.Vision.AlignToGoalAuton;
import frc.robot.subsystems.Climber;
import frc.robot.subsystems.ClimberHooks;
import frc.robot.subsystems.Drivebase;
import frc.robot.subsystems.FloorIntake;
import frc.robot.subsystems.LED;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.Tower;
import frc.robot.subsystems.tracking.LimelightInterface;
import frc.robot.subsystems.tracking.PhotonVisionInterface;
import frc.robot.utilities.XboxController;

/**
 * This class is where the bulk of the robot should be declared. Since
 * Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in
 * the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of
 * the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  //
  public enum ClimbSelector {
    ZERO,
    ONE,
    TWO,
    THREE,
    FOUR,
    FIVE,
    SIX
  }

  public static int climbStage = 0;

  Drivebase mDrivebase = Drivebase.getInstance();
  PhotonVisionInterface mPhotonVisionInterface = PhotonVisionInterface.getInstance();
  LED mLED = LED.getInstance();
  Tower mTower = Tower.getInstance();
  Shooter mShooter = Shooter.getInstance();
  Climber mClimber; //= Climber.getInstance();
  ClimberHooks mClimberHooks; //= ClimberHooks.getInstance();

  FloorIntake mFloorIntake = FloorIntake.getInstance();

  private final XboxController mDriverController = new XboxController(
      Constants.ControllerInputs.DRIVER_CONTROLLER_PORT);
  private final XboxController mOperatorController = new XboxController(
      Constants.ControllerInputs.OPERATOR_CONTROLLER_PORT);
  // private final XboxController mProgrammingController = new XboxController(
  // Constants.ControllerInputs.PROGRAMMER_CONTROLLER_PORT);

  private SendableChooser<Command> mAutonomousChooser;

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   * 
   * @throws FileNotFoundException
   */
  public RobotContainer() throws FileNotFoundException {

    try {
      AutoPaths autoPaths = new AutoPaths();
    } catch (FileNotFoundException e) {
      e.printStackTrace();
    }

    // Real ones
    mAutonomousChooser = new SendableChooser<Command>();
    mAutonomousChooser.addOption("Three All High", new ThreeBallHighTest(mDrivebase, mFloorIntake, mTower, mShooter));
    mAutonomousChooser.addOption("Drive And Turn", new DriveAndTurn(mDrivebase, mFloorIntake, mTower, mShooter));
    mAutonomousChooser.addOption("TwoBall", new TwoBallHigh(mDrivebase, mFloorIntake, mTower, mShooter));
    mAutonomousChooser.addOption("Chase Tag", new ChaseTagCommand(mPhotonVisionInterface, mDrivebase));

  
    // mAutonomousChooser.addOption();

    // Test ones
    // mAutonomousChooser.addOption("Example Auton", new ExampleAuton(mDrivebase));
    // mAutonomousChooser.addOption("One Meter Forward", new
    // OneMeterForward(mDrivebase));
    // mAutonomousChooser.addOption("Test Constant Auton", new
    // TestConstantAuton(mDrivebase));

    Shuffleboard.getTab("Autonomous").add(mAutonomousChooser);

    if (Robot.blueRobot) {
      mClimber = Climber.getInstance();
      mClimberHooks = ClimberHooks.getInstance();
      mLED.setDefaultCommand(new RunLED(mLED, mTower, mShooter,
          mPhotonVisionInterface, mClimberHooks, mClimber, () -> mDriverController.getRightTrigger()));
    }

    mDrivebase.setDefaultCommand(new FieldOrientedDrive(
        mDrivebase,
        () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Velocity") 
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getLeftXAxis(), "Y Velocity")
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getDeadbandedRightXAxis(Constants.ControllerInputs.DEADBAND) / 2.0,
            "Rotation")
            * Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));
    
    // mDrivebase.setDefaultCommand(new DriveFalconOnly(
    //     mDrivebase,
    //     () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Veloctiy")));

    // mDrivebase.setDefaultCommand(
    // new DriveWithHeading(
    // mDrivebase,
    // () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Velocity")
    // * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(mDriverController.getLeftXAxis(), "Y Velocity")
    // * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(mDriverController.getRightXAxis(), "Rotation X"),
    // () -> -modifyAxis(mDriverController.getRightYAxis(), "Rotation Y")));

    // mClimber.setDefaultCommand(
    // new RunClimber(
    // mClimber,
    // () -> mProgrammingController.getDeadbandedLeftYAxis(0.05) / 2.0));

    // Configure the button bindings
    configureButtonBindings();
  }

  public ClimbSelector getClimbStage() {
    // ClimbSelector lastStage = mClimber.climbStage;

    switch (mClimber.climbStage) {
      case ZERO:
        mClimber.climbStage = ClimbSelector.ONE;
        break;
      case ONE:
        mClimber.climbStage = ClimbSelector.TWO;
        break;
      case TWO:
        mClimber.climbStage = ClimbSelector.THREE;
        break;
      case THREE:
        mClimber.climbStage = ClimbSelector.FOUR;
        break;
      case FOUR:
        mClimber.climbStage = ClimbSelector.FIVE;
        break;
      case FIVE:
        mClimber.climbStage = ClimbSelector.FIVE;
        break;

    }
    // mClimber.climbStage = lastStage;
    // SmartDashboard.putString("Testing/climbing stage selector",
    // mClimber.climbStage.toString());
    return mClimber.climbStage;
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be
   * created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing
   * it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {

    Button mDriverLeftBumper = new Button(() -> mDriverController.getLB());
    mDriverLeftBumper.whenHeld(new AlignToGoal(
      mDrivebase,
      () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Velocty")
          * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(mDriverController.getLeftXAxis(), "Y Velocity")
          * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
      () -> -modifyAxis(mDriverController.getDeadbandedRightXAxis(Constants.ControllerInputs.DEADBAND),
          "Rotation")
          * Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, false));

    Button mDriverStartButton = new Button(() -> mDriverController.getStartButton());
    mDriverStartButton.whenPressed(new ResetAll(mDrivebase));
    
    Button mDriverBackButton = new Button(() -> mDriverController.getBackButton());
    mDriverBackButton.whenPressed(new ResetAll(mDrivebase));

    Button mDriverRightTrigger = new Button(() -> mDriverController.getRightTrigger());
    mDriverRightTrigger.whenHeld(new AlignToGoal(
        mDrivebase,
        () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Velocty")
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getLeftXAxis(), "Y Velocity")
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getDeadbandedRightXAxis(Constants.ControllerInputs.DEADBAND),
            "Rotation")
            * Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND, true)
            .andThen(new DiamondFormation(
              mDrivebase)));

    Button mDriverLeftTriggerButton = new Button(() -> mDriverController.getLeftTrigger());
    mDriverLeftTriggerButton.whenHeld(new ReducedSpeed(
        mDrivebase,
        () -> -modifyAxis(mDriverController.getDeadbandedLeftYAxis(0.05), "X Axis")
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getDeadbandedLeftXAxis(0.05), "Y Axis")
            * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
        () -> -modifyAxis(mDriverController.getDeadbandedRightXAxis(0.05) / 2.0, "Rotation")
            * Constants.Drivebase.MAX_ANGULAR_VELOCITY_RADIANS_PER_SECOND));

    // Button mDriverBButton = new Button(() -> mDriverController.getBButton());
    // mDriverBButton.whenPressed(new TurnToAngle(mDrivebase, 90.0));
    // Button mDriverYButton = new Button(() -> mDriverController.getYButton());
    // mDriverYButton.whenPressed(new TurnToAngle(mDrivebase, -45.0));

    Button mDriverYButton = new Button(() -> mDriverController.getYButton());
    mDriverYButton.whenPressed(new AlignToGoalAuton(mDrivebase));

    Button mDriverBButton = new Button(() -> mDriverController.getBButton());
    mDriverBButton.whenPressed(new ShootHighAuto(mShooter, mTower, true, () -> Constants.Shooter.DEFAULT_HIGH_RPM).withTimeout(2.0));
   

    Button mDriverAButton = new Button(() -> mDriverController.getAButton());
    mDriverAButton.whenPressed( new TurnToAngle(mDrivebase, 30.0));

    // Button mDriverBackButton = new Button(() ->
    // mDriverController.getBackButton());
    // mDriverBackButton.whenPressed(new InvertTurnMotors(mDrivebase));

    // Button mOperatorXButton = new Button(() -> mOperatorController.getXButton());
    // mOperatorXButton.whenPressed(new ShoulderSetPoint(mClimber, 5.0));
    // mOperatorXButton.whenReleased(new ShoulderSetPoint(mClimber, -26.0));

    // Button mDriverYButton = new Button(() -> mDriverController.getYButton());
    // mDriverYButton.whenHeld(new AlignByTranslation(
    // mDrivebase,
    // () -> mDriverController.getLeftYAxis(),
    // () -> mDriverController.getLeftXAxis()));

    // Button mDriverRightTrigger = new Button(() ->
    // mDriverController.getRightTrigger());
    // mDriverRightTrigger.whileHeld(new OffsetRobotCenter(
    // mDrivebase,
    // () -> -modifyAxis(mDriverController.getLeftYAxis(), "X Velocty")
    // * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> -modifyAxis(mDriverController.getLeftXAxis(), "Y Velocity")
    // * Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND,
    // () -> mDriverController.getRightXAxis(),
    // () -> Units.inchesToMeters(mLimelightInterface.getDistance())));

    // Button mDriverXButton = new Button(() -> mDriverController.getXButton());
    // mDriverXButton.whenHeld(new OffsetRobotCenter(
    // mDrivebase,
    // () -> 0.0,
    // () -> 0.0,
    // () -> mDriverController.getRightXAxis(),
    // () -> Units.inchesToMeters(Constants.Drivebase.JUKE_DISTANCE)));
    // Button mDriverXButton = new Button(() -> mDriverController.getXButton());
    // mDriverXButton.whileHeld(new TurnToAngle(mDrivebase, () -> 45.0));

    // Button mOperatorYButton = new Button(() -> mOperatorController.getYButton());
    // mOperatorYButton.whileHeld(new ShooterRPM(
    // mShooter,
    // mTower,

    // () -> true, // mOperatorController.getLeftTrigger(),
    // true));
    if (Robot.blueRobot) {
      Button mOperatorBackButton = new Button(() -> mOperatorController.getBackButton());
      // mOperatorBackButton.whenPressed(new AdvanceClimb(mClimber, mClimberHooks,
      // mDrivebase, () -> mClimber.climbStage));
      mOperatorBackButton.whenPressed(
          new SelectCommand(
              Map.ofEntries(
                  // Map.entry(ClimbSelector.ZERO, new InstantCommand()),
                  Map.entry(ClimbSelector.ONE, new PrepareForMidBar(mClimberHooks, mClimber)),
                  Map.entry(ClimbSelector.TWO, new LockOnMidBar(mClimberHooks, mClimber, mDrivebase)),
                  Map.entry(ClimbSelector.THREE, new RotateToHighBar(mClimber, mClimberHooks)),
                  Map.entry(ClimbSelector.FOUR, new RotateToTraversal(mClimber, mClimberHooks)),
                  Map.entry(ClimbSelector.FIVE, new LockOnToTraversal(mClimber, mClimberHooks)),
                  Map.entry(ClimbSelector.SIX, new InstantCommand())),
              () -> mClimber.climbStage));
    Button mOperatorStartButton = new Button(() -> mOperatorController.getStartButton());
    mOperatorStartButton.whenPressed(new RequireClimberAndHooks(mClimber, mClimberHooks));
    }

    Button mOperatorAButton = new Button(() -> mOperatorController.getAButton());
    mOperatorAButton.whenHeld(new RunFloorIntake(mFloorIntake, mTower));

    // color = DriverStation.getInstance().getAlliance();
    // if(color == DriverStation.Alliance.Blue){
    // mIsRedSide = false;
    // Lighting.getInstance().setAllianceLightBlue();
    // autonList = Constants.TrajectoryFiles.BLUE_AUTONS;
    // } else if (color == DriverStation.Alliance.Red){
    // mIsRedSide = true;
    // Lighting.getInstance().setAllianceLightRed();
    // autonList = Constants.TrajectoryFiles.RED_AUTONS;
    // }

    DriverStation.Alliance color = DriverStation.getAlliance();

    // Button mOperatorRightTrigger = new Button(() ->
    // mOperatorController.getRightTrigger());
    // mOperatorRightTrigger.whenHeld(new SpitOutBadBalls(mTower,
    // mShooter,
    // mFloorIntake,
    // Tower.BallState.Blue));
    Button mOperatorXButton = new Button(() -> mOperatorController.getXButton());
    mOperatorXButton.whenHeld(new SpitOutBadBalls(mTower,
        mShooter,
        mFloorIntake,
        // Tower.BallState.Blue));
        (color == DriverStation.Alliance.Blue) ? Tower.BallState.Blue
            : Tower.BallState.Red));

    Button mOperatorBButton = new Button(() -> mOperatorController.getBButton());
    mOperatorBButton.whenPressed(new ToggleFloorIntakeState(mFloorIntake));

    Button mOperatorLeftTrigger = new Button(() -> mOperatorController.getLeftTrigger());
    mOperatorLeftTrigger.whenHeld(new ShooterRPMHigh(mShooter, mTower, true, () -> Constants.Shooter.DEFAULT_HIGH_RPM, false));// SmartDashboard.getNumber("Shooter/RPM
    // SetPoint", 1500)));

    Button mOperatorRightTrigger = new Button(() -> mOperatorController.getRightTrigger());
    // mOperatorRightTrigger.whileHeld(new ShootRPMSetPoint(mShooter, 1500,
    // mTower));// SmartDashboard.getNumber("Shooter/RPM
    // SetPoint", 1500)));

    mOperatorRightTrigger.whenHeld(new ShooterRPMLow(mShooter, mTower, true, () -> Constants.Shooter.DEFAULT_LOW_RPM, false));// SmartDashboard.getNumber("Shooter/RPM

    Button mOperatorYButton = new Button(() -> mOperatorController.getYButton());
    mOperatorYButton.whileHeld(new RunFloorIntakeOut(mFloorIntake, mTower));

    // Trigger leftTrigger = new Trigger(() ->
    // mOperatorController.getLeftTrigger());
    // leftTrigger.whileActiveContinuous());

    // Button mOperatorXButton = new Button(() -> mOperatorController.getXButton());
    // mOperatorXButton.whenHeld(new RunFloorIntake(mFloorIntake));

    // Button mOperatorAButton = new Button(() -> mOperatorController.getAButton());
    // mOperatorAButton.whenHeld(new RunShooter(mShooter, () ->
    // mOperatorController.getDeadbandedLeftYAxis(0.05)));

    // Button mOperatorAButton = new Button(() -> mOperatorController.getAButton());
    // mOperatorAButton.whenPressed(new SetBothHookPosition(mClimber, ()-> 0.0, ()
    // -> 0.0));

    // Button mOperatorXButton = new Button(() -> mOperatorController.getXButton());
    // mOperatorXButton.whenPressed(new SetBothHookPosition(mClimber, () -> 180.0,
    // () -> 180.0));

    // Button mOperatorBButton = new Button(() -> mOperatorController.getBButton());
    // mOperatorBButton.whenPressed(new SetBothHookPosition(mClimber, () -> 45.0, ()
    // -> 45.0));

    // Button mOperatorYButton = new Button(() -> mOperatorController.getYButton());
    // mOperatorYButton.whenPressed(new SetBothHookPosition(mClimber, () -> 135.0,
    // () -> 135.0));

    // NOTE: using whileHeld until we are confident they are working, then we can
    // try with whenPressed
    // Button mProgrammerStartButton = new Button(() ->
    // mProgrammingController.getStartButton());
    // mProgrammerStartButton.whileHeld(new DriveToMidBar(mClimber, mClimberHooks,
    // mDrivebase));

    // Button mProgrammerAButton = new Button(() ->
    // mProgrammingController.getAButton());
    // mProgrammerAButton.whileHeld(new SetLongSidePosition(mClimberHooks, 90));

    // Button mProgrammerBButton = new Button(() ->
    // mProgrammingController.getBButton());
    // mProgrammerBButton.whileHeld(new SetShortSidePosition(mClimberHooks, 90));

    // Button mProgrammerYButton = new Button(() ->
    // mProgrammingController.getYButton());
    // mProgrammerYButton.whileHeld(new GoToClosedPosition(mClimberHooks, () ->
    // true, () -> false));

    // Button mProgrammerXButton = new Button(() ->
    // mProgrammingController.getXButton());
    // mProgrammerXButton.whileHeld(new GoToClosedPosition(mClimberHooks, () ->
    // false, () -> true));

    // Button mProgrammerBackButton = new Button(() ->
    // mProgrammingController.getBackButton());
    // mProgrammerBackButton.whileHeld(new ResetHookReferencePoint(mClimberHooks));

    // if (Robot.blueRobot) {
    //   Button mDriverBackButton = new Button(() -> mDriverController.getBackButton());
    //   mDriverBackButton.whileHeld(new ResetHookReferencePoint(mClimberHooks)).whileHeld(new ResetArmEncoders(mClimber));
    // }

    // mOperatorXButton.whileHeld(
    // new SetColor(mLED)
    // );

    // Button mOperatorAButton = new Button(() -> mOperatorController.getAButton());
    // mOperatorAButton.whenHeld(new ShootAllBalls(
    // mShooter,
    // mTower,
    // () -> mOperatorController.getLeftTrigger(),
    // true
    // )
    // )

    // LimelightInterface limelight = new LimelightInterface();

    // Button mProgrammerLBButton = new Button(() -> mProgrammerController.getLB());
    // mProgrammerLBButton.whileHeld(
    // new RunHook(mClimberHooks, () ->
    // mProgrammerController.getDeadbandedRightYAxis(0.05) / 4.0,
    // () -> 0.0))

    if (Robot.blueRobot) {
      Button mDriverRButton = new Button(() -> mDriverController.getRB());
      mDriverRButton.whenPressed(new ShoulderSetPoint(mClimber, -26.0))
          .whenReleased(new ShoulderSetPoint(mClimber, 10.0));
  

      Button mOperatorLBButton = new Button(() -> mOperatorController.getLB());
      mOperatorLBButton.whileHeld(
          new RunHook(mClimberHooks, () -> mOperatorController.getDeadbandedRightYAxis(0.05) / 4.0,
              () -> 0.0))
          .whileHeld(new RunClimber(
              mClimber,
              () -> mOperatorController.getDeadbandedLeftYAxis(0.05) / 2.0));

      Button mOperatorRBButton = new Button(() -> mOperatorController.getRB());
      mOperatorRBButton.whileHeld(
          new RunHook(mClimberHooks, () -> 0.0, () -> mOperatorController.getDeadbandedRightYAxis(0.05) / 4.0))
          .whileHeld(new RunClimber(
              mClimber,
              () -> mOperatorController.getDeadbandedLeftYAxis(0.05) / 2.0));
    }
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   * @throws FileNotFoundException
   */
  public Command getAutonomousCommand() {
    // return new ResetAll(mDrivebase).andThen(new FollowTrajectory(mDrivebase, AutoPaths.meterForward, "One Meter Forwards"));
    // try {
    // return new TestConstantAuton(mDrivebase);
    // } catch (FileNotFoundException e) {
    // // e.printStackTrace();
    // return new InstantCommand(() -> System.out.println("Autonomous didn't run"));
    // }
    // return new OneMeterForward(mDrivebase);
    // return new
    // return new OneBall(mDrivebase);
    // An ExampleCommand will run in autonomous
    // try {
    // // return new FourBall(mDrivebase);
    // return new ThreeBall(mDrivebase, mFloorIntake, mShooter, mTower);
    // // // return new TwoBall(mDrivebase);
    // } catch (FileNotFoundException e) {
    // e.printStackTrace();
    // return new InstantCommand();
    // }
    return mAutonomousChooser.getSelected();
  }

  private static double modifyAxis(double value, String purpose) {
    // System.out.print("Pre-Value: " + value + ". ");

    // Deadband
    if (Math.abs(value) <= Constants.ControllerInputs.DEADBAND) {
      SmartDashboard.putNumber(purpose, 0.0);

      return 0.0;
    }
    // Cube the axis
    // value = Math.copySign(value * value, value);
    value = value * value * value;
    SmartDashboard.putNumber(purpose, value);


    // System.out.println("Value: " + value + ". Purpose: " + purpose);
    // if (value < 0.0) value = -1.0;
    // else if (value > 0.0) value = 1.0;
    return value;
  }
}