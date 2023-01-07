// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.Autonomous;

import java.io.File;
import java.io.IOException;
import java.nio.file.Path;
import java.util.ArrayList;
import java.util.List;

import javax.print.event.PrintJobListener;

import org.ejml.masks.FMaskFactory;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.math.trajectory.Trajectory.State;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Robot;

/** Add your docs here. */
public class LoadTrajectoryFromPathweaver {
    public Trajectory traj;
    public LoadTrajectoryFromPathweaver() throws IOException {
        List<String> files = new ArrayList<>();
        files.add("TestPath1.wpilib.json");

        
        Pose2d prev_pose = new Pose2d();
        List<Pose2d> tr = new ArrayList<>();


        for (String fname : files) {
            Trajectory trajectory = TrajectoryUtil.fromPathweaverJson(Filesystem.getDeployDirectory().toPath().resolve(fname));

            

            List<Trajectory.State> trajectoryPoints = trajectory.getStates();
            trajectory = trajectory.relativeTo(prev_pose.equals(new Pose2d()) ? trajectoryPoints.get(0).poseMeters : prev_pose);
            // if prev pose not set then very first point of path... need to set it to first point
            
            SmartDashboard.putString("Prev_pose", prev_pose.equals(new Pose2d()) ? trajectoryPoints.get(0).poseMeters.toString() : prev_pose.toString());
            double xi = trajectoryPoints.get(0).poseMeters.getX() + prev_pose.getX();
            double yi = trajectoryPoints.get(0).poseMeters.getY() + prev_pose.getY();

            double thetai = trajectoryPoints.get(0).poseMeters.getRotation().getDegrees() - prev_pose.getRotation().getDegrees();

            //for (int i )


            int last_pos = trajectoryPoints.size() - 1;

            double xf = trajectoryPoints.get(last_pos).poseMeters.getX() + prev_pose.getX();
            double yf = trajectoryPoints.get(last_pos).poseMeters.getY() + prev_pose.getY();

            double thetaf = trajectoryPoints.get(last_pos).poseMeters.getRotation().getDegrees() - prev_pose.getRotation().getDegrees();
            
            tr.add(new Pose2d(new Translation2d(xi, yi), Rotation2d.fromDegrees(thetai)));
            tr.add(new Pose2d(new Translation2d(xf, yf), Rotation2d.fromDegrees(thetaf)));
            prev_pose = trajectoryPoints.get(last_pos).poseMeters;


        }

        traj = TrajectoryGenerator.generateTrajectory(tr, new TrajectoryConfig(2.0, 1.0));
        // if (Robot.logging)
        // SmartDashboard.putString("Pathweaver Path", tr.toString());
        
    }
}
