// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.io.FileNotFoundException;
import java.util.ArrayList;

import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import frc.robot.Constants;
import frc.robot.subsystems.Drivebase;

/** Add your docs here. */
public class AutoPaths {
    public static LoadTrajectoryFromFile TestPathPart1;
    public static LoadTrajectoryFromFile TestPathPart2;
    public static LoadTrajectoryFromFile TestPathPart3;
    public static LoadTrajectoryFromFile TestPathPart4;

    public static LoadTrajectoryFromFile ThreeBallPathPart1;
    public static LoadTrajectoryFromFile ThreeBallPathPart2;
    public static LoadTrajectoryFromFile ThreeBallPathPart3;

    public static LoadTrajectoryFromFile ThreeBallHighPathPart2a;
    public static LoadTrajectoryFromFile ThreeBallHighPathPart2b;
    public static LoadTrajectoryFromFile ThreeBallHighPathPart3;
    public static LoadTrajectoryFromFile ThreeBallHighPathPart4;

    // public static LoadTrajectoryFromFile HideBallPartOne;
    // public static LoadTrajectoryFromFile StealCenterBall;
    // public static LoadTrajectoryFromFile StealOuterBall;



    public static LoadTrajectoryFromFile TwoBallPathPart1;
    public static LoadTrajectoryFromFile TwoBallPathPart2;
    public static LoadTrajectoryFromFile TwoBallPathPart3;

    public static LoadTrajectoryFromFile OneBallPath;

    public static LoadTrajectoryFromFile OneMeterForward;


    public static LoadTrajectoryFromFile FourBallPathPart1;
    public static LoadTrajectoryFromFile FourBallPathPart2;
    public static LoadTrajectoryFromFile FourBallPathPart3;
    public static LoadTrajectoryFromFile FourBallPathPart4;
    public static LoadTrajectoryFromFile FourBallPathPart5;
    public static LoadTrajectoryFromFile FourBallPathPart6;

    public static LoadTrajectoryFromFile TwoBallHighPathPart1;
    public static LoadTrajectoryFromFile TwoBallHighPathPart2;

    public static LoadTrajectoryFromFile FiveBallToHP;
    public static LoadTrajectoryFromFile FiveBallFromHP;


    public static LoadTrajectoryFromFile ThreeBallHighPath;

    public static LoadTrajectoryFromFile ShootAndMovePath;
    
    public static LoadTrajectoryFromFile TwoBallToHP;
    public static LoadTrajectoryFromFile TwoBallFromHP;
    public static LoadTrajectoryFromFile MARCHAMPS_BACKWARD;
    public static LoadTrajectoryFromFile MARCHAMPS_LEFT;

    
    public static Trajectory testpart1;
    public static Trajectory testpart2;
    public static Trajectory testpart3;
    public static Trajectory testpart4;

    public static Trajectory threeballpart1;
    public static Trajectory threeballpart2;
    public static Trajectory threeballpart3;
    
    public static Trajectory threeballhighpart2a;
    public static Trajectory threeballhighpart2b;
    public static Trajectory threeballhighpart3;
    public static Trajectory threeballhighpart4;


    public static Trajectory twoballpart1;
    public static Trajectory twoballpart2;
    public static Trajectory twoballpart3;

    public static Trajectory twoballhighpart1;
    public static Trajectory twoballhighpart2;

    
    public static LoadTrajectoryFromFile HideBallPathPart1;
    public static LoadTrajectoryFromFile StealCenterBallPath;
    public static LoadTrajectoryFromFile StealOuterBallPath;

    
    public static Trajectory hideballpart1;
    public static Trajectory stealouterball;
    public static Trajectory stealcenterball;

    public static Trajectory oneballpath;

    public static Trajectory meterForward;

    public static Trajectory fourballpart1;
    public static Trajectory fourballpart2;
    public static Trajectory fourballpart3;
    public static Trajectory fourballpart4;
    public static Trajectory fourballpart5;
    public static Trajectory fourballpart6;

    public static Trajectory shootandmove;

    public static Trajectory twoballtohp;
    public static Trajectory twoballfromhp;

    public static Trajectory threeballhigh;

    public static Trajectory fiveballtohppath;
    public static Trajectory fiveballfromhppath;

    public static Trajectory marchamps_backward;
    public static Trajectory marchamps_left;



    public static LoadTrajectoryFromFile ThreeBallPart4;
    public static Trajectory threeballpart4;

    Drivebase mDrivebase = Drivebase.getInstance();
    TrajectoryConfig defaulTrajectoryConfig = mDrivebase.getTrajectoryConfig();

    // Used to start from no velocity and end at any velocity
    TrajectoryConfig startMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

        // Used to start and end in one path
    TrajectoryConfig singlePath = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

        // Used to start and end in one path
    TrajectoryConfig reversedSinglePath = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    // Used to start and end at any velocity
    TrajectoryConfig keepMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    // Used to start at any velocity and end stopped
    TrajectoryConfig stopMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    // Used to start from stopped and end at any velocity, while going backwards
    TrajectoryConfig reversedStartMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);
    // Used to start from stopped and end at any velocity, while going backwards
    TrajectoryConfig reversedKeepMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);
    // Used to start from stopped and end at any velocity, while going backwards
    TrajectoryConfig reversedStopMovingConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    // Used to start and end in one path
    TrajectoryConfig singlePathFast = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

        // Used to start and end in one path
    TrajectoryConfig reversedSinglePathFast = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    TrajectoryConfig threeBallPartOneConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    TrajectoryConfig threeBallPartTwoAConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);
    
    TrajectoryConfig threeBallPartTwoBConfig = new TrajectoryConfig(
        Constants.Auto.MAX_AUTO_VELOCITY, 
        Constants.Drivebase.MAX_ACCELERATION);

    public static ArrayList<PathPlannerTrajectory> threeBallTest = PathPlanner.loadPathGroup("3BallTest", Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND, Constants.Drivebase.MAX_ACCELERATION);
    public static PathPlannerTrajectory driveAndTurn = PathPlanner.loadPath("DriveAndTurn", Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND, Constants.Drivebase.MAX_ACCELERATION);
    public static PathPlannerTrajectory twoBall = PathPlanner.loadPath("Two Ball", Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND, Constants.Drivebase.MAX_ACCELERATION);
    public static ArrayList<PathPlannerTrajectory> fiveBall = PathPlanner.loadPathGroup("FiveBall", Constants.Drivebase.MAX_VELOCITY_METERS_PER_SECOND, Constants.Drivebase.MAX_ACCELERATION);


    // TrajectoryConfig reversedStartStopConfig = new TrajectoryConfig(
    //     Constants.Auto.MAX_AUTO_VELOCITY, 
    //     Constants.Drivebase.MAX_ACCELERATION);

    public AutoPaths() throws FileNotFoundException {
        // Loading autonomous paths
        startMovingConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        startMovingConfig.setEndVelocity(Constants.Auto.MAX_AUTO_VELOCITY);

        keepMovingConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        keepMovingConfig.setEndVelocity(Constants.Auto.MAX_AUTO_VELOCITY);

        stopMovingConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        stopMovingConfig.setEndVelocity(0.0);

        reversedStartMovingConfig.setStartVelocity(0.0);
        reversedStartMovingConfig.setEndVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        reversedStartMovingConfig.setReversed(true);

        // reversedStartMovingConfig.setStartVelocity(0.0);
        // reversedStartMovingConfig.setEndVelocity(0.0);
        // reversedStartMovingConfig.setReversed(true);

        reversedKeepMovingConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        reversedKeepMovingConfig.setEndVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        reversedKeepMovingConfig.setReversed(true);

        reversedStopMovingConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        reversedStopMovingConfig.setEndVelocity(0.0);
        reversedStopMovingConfig.setReversed(true);

        singlePath.setStartVelocity(0.0);
        singlePath.setEndVelocity(0.0);

        reversedSinglePath.setStartVelocity(0.0);
        reversedSinglePath.setEndVelocity(0.0);
        reversedSinglePath.setReversed(true);

        
        singlePathFast.setStartVelocity(0.0);
        singlePathFast.setEndVelocity(0.0);

        reversedSinglePathFast.setStartVelocity(0.0);
        reversedSinglePathFast.setEndVelocity(0.0);
        reversedSinglePathFast.setReversed(true);

        threeBallPartOneConfig.setStartVelocity(0.0);
        threeBallPartOneConfig.setEndVelocity(0.5);
        threeBallPartOneConfig.setReversed(false);

        threeBallPartTwoAConfig.setStartVelocity(0.5);
        threeBallPartTwoAConfig.setEndVelocity(0.0);
        threeBallPartTwoAConfig.setReversed(false);

        threeBallPartTwoBConfig.setStartVelocity(Constants.Auto.MAX_AUTO_VELOCITY);
        threeBallPartTwoBConfig.setEndVelocity(0.0);
        threeBallPartTwoBConfig.setReversed(false);
   
        // Load trajectories from a file
        // Will throw an error if the trajectory is not present
        TestPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/PathPart1.txt");
        TestPathPart2 = new LoadTrajectoryFromFile("/home/lvuser/deploy/PathPart2.txt");
        TestPathPart3 = new LoadTrajectoryFromFile("/home/lvuser/deploy/PathPart3.txt");
        TestPathPart4 = new LoadTrajectoryFromFile("/home/lvuser/deploy/PathPart4.txt");

        ThreeBallPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBall1.txt");
        ThreeBallPathPart2 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBall2.txt");
        ThreeBallPathPart3 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBall3.txt");
        ThreeBallPart4 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBall4.txt");

        ThreeBallHighPathPart2a = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBallHighPart2a.txt"); 
        ThreeBallHighPathPart2b = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBallHighPart2b.txt");
        ThreeBallHighPathPart3 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBallHighPart3.txt");
        ThreeBallHighPathPart4 = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBallHighPart4.txt");

        MARCHAMPS_BACKWARD = new LoadTrajectoryFromFile("/home/lvuser/deploy/MAR/1.txt");
        MARCHAMPS_LEFT = new LoadTrajectoryFromFile("/home/lvuser/deploy/MAR/2.txt");

        marchamps_backward = MARCHAMPS_BACKWARD.getTrajectory(reversedSinglePath);
        marchamps_left = MARCHAMPS_LEFT.getTrajectory(singlePath);

        
        HideBallPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/ShootTwo.txt");
        StealCenterBallPath = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/LeftSteal.txt");
        StealOuterBallPath = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/RightSteal.txt");

        
        hideballpart1 = HideBallPathPart1.getTrajectory(singlePath);
        stealcenterball = StealCenterBallPath.getTrajectory(reversedSinglePath);
        stealouterball = StealOuterBallPath.getTrajectory(reversedSinglePath);

        TwoBallPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallAuto/2BallAuto1.txt");
        TwoBallPathPart2 = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallAuto/2BallAuto2.txt");
        TwoBallPathPart3 = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallAuto/2BallAuto3.txt");

        OneBallPath = new LoadTrajectoryFromFile("/home/lvuser/deploy/OneBallAuto/OneBallAuto.txt");

        OneMeterForward = new LoadTrajectoryFromFile("/home/lvuser/deploy/TestAuto/OneMeterForward.txt");

        FourBallPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto1.txt");
        FourBallPathPart2 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto2.txt");
        FourBallPathPart3 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto3.txt");
        FourBallPathPart4 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto4.txt");
        FourBallPathPart5 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto5.txt");
        FourBallPathPart6 = new LoadTrajectoryFromFile("/home/lvuser/deploy/FourBallAuto/4BallAuto6.txt");

        TwoBallHighPathPart1 = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallHigh/TwoBallHigh1.txt");
        TwoBallHighPathPart2 = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallHigh/TwoBallHigh2.txt");
        
        TwoBallToHP = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallHigh/TwoBallHP.txt");
        TwoBallFromHP = new LoadTrajectoryFromFile("/home/lvuser/deploy/TwoBallHigh/TwoBallBack.txt");

        ThreeBallHighPath = new LoadTrajectoryFromFile("/home/lvuser/deploy/ThreeBallAuto/ThreeBallHigh.txt");

        ShootAndMovePath = new LoadTrajectoryFromFile("/home/lvuser/deploy/MoveAwayAuto/MoveAway.txt");

        FiveBallToHP = new LoadTrajectoryFromFile("/home/lvuser/deploy/FiveBallAuto/FiveBallToHP.txt");
        FiveBallFromHP = new LoadTrajectoryFromFile("/home/lvuser/deploy/FiveBallAuto/FiveBallFromHP.txt");

        // HideBallPartOne = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/ShootTwo.txt");
        // StealCenterBall = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/LeftSteal.txt");
        // StealOuterBall = new LoadTrajectoryFromFile("/home/lvuser/deploy/HideBall/RightSteal.txt");

        // hideballpart1 = HideBallPartOne.getTrajectory(config);

        testpart1 = TestPathPart1.getTrajectory(startMovingConfig);
        testpart2 = TestPathPart2.getTrajectory(keepMovingConfig);
        testpart3 = TestPathPart3.getTrajectory(keepMovingConfig);
        testpart4 = TestPathPart4.getTrajectory(stopMovingConfig);

        // threeballpart1 = ThreeBallPathPart1.getTrajectory(reversedStartMovingConfig);
        // threeballpart2 = ThreeBallPathPart2.getTrajectory(stopMovingConfig);
        threeballpart1 = ThreeBallPathPart1.getTrajectory(threeBallPartOneConfig);
        threeballpart2 = ThreeBallPathPart2.getTrajectory(threeBallPartTwoAConfig);
        threeballpart3 = ThreeBallPathPart3.getTrajectory(stopMovingConfig);
        threeballpart4 = ThreeBallPart4.getTrajectory(reversedStartMovingConfig);

        threeballhighpart2a = ThreeBallHighPathPart2a.getTrajectory(threeBallPartTwoAConfig);
        threeballhighpart2b = ThreeBallHighPathPart2b.getTrajectory(threeBallPartTwoBConfig);
        threeballhighpart3 = ThreeBallHighPathPart3.getTrajectory(singlePath);
        threeballhighpart4 = ThreeBallHighPathPart4.getTrajectory(singlePath);


        twoballpart1 = TwoBallPathPart1.getTrajectory(reversedSinglePath);
        twoballpart2 = TwoBallPathPart2.getTrajectory(singlePath);
        twoballpart3 = TwoBallPathPart3.getTrajectory(reversedStartMovingConfig);

        twoballhighpart1 = TwoBallHighPathPart1.getTrajectory(reversedSinglePath);
        twoballhighpart2 = TwoBallHighPathPart2.getTrajectory(reversedSinglePath);

        twoballtohp = TwoBallToHP.getTrajectory(singlePath);
        twoballfromhp = TwoBallFromHP.getTrajectory(reversedSinglePath);
        
        oneballpath = OneBallPath.getTrajectory(reversedStartMovingConfig);

        meterForward = OneMeterForward.getTrajectory(defaulTrajectoryConfig);

        fourballpart1 = FourBallPathPart1.getTrajectory(reversedStartMovingConfig);
        fourballpart2 = FourBallPathPart2.getTrajectory(stopMovingConfig);
        // fourballpart3 = FourBallPathPart3.getTrajectory(stopMovingConfig);
        fourballpart4 = FourBallPathPart4.getTrajectory(singlePathFast);
        // fourballpart5 = FourBallPathPart5.getTrajectory(reversedStopMovingConfig);
        fourballpart6 = FourBallPathPart6.getTrajectory(reversedSinglePathFast);
        
        fiveballtohppath = FiveBallToHP.getTrajectory(reversedSinglePath);
        fiveballfromhppath = FiveBallFromHP.getTrajectory(reversedSinglePath);


        threeballhigh = ThreeBallHighPath.getTrajectory(stopMovingConfig);

        shootandmove = ShootAndMovePath.getTrajectory(reversedStartMovingConfig);

    }
}
