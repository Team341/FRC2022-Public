package frc.robot.commands.Autonomous;

import java.io.File;
import java.io.FileNotFoundException;
import java.util.ArrayList;
import java.util.List;
import java.util.Scanner;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class LoadTrajectoryFromFile {
    public List<Translation2d> interiorPoints = new ArrayList<Translation2d>();
    public Pose2d start, end;
    public boolean firstPointset = false;

    public LoadTrajectoryFromFile(String filePath) throws FileNotFoundException {
        File file = new File(filePath);
        Scanner sc = new Scanner(file);

        // Is this necessary? If not is it computationally expensive?
        interiorPoints.clear();

        // Iterate through the file and add each point to the list
        while (sc.hasNext()) {
            String line = sc.nextLine();
            String[] values = line.split(", ");

            // Is there some way to move this out of the while loop? TODO
            // pose2d
            if (!firstPointset) { // Sets first point of autonomous path and uses pose2d with rotation
                start = new Pose2d(
                        Units.inchesToMeters(Double.parseDouble(values[0])),
                        Units.inchesToMeters(Double.parseDouble(values[1])),
                        Rotation2d.fromDegrees(Double.parseDouble(values[2])));
                firstPointset = true;
            } 
            // Can this be moved out of the while loop somehow as well?
            else if (!sc.hasNext()) {
                end = new Pose2d(
                        Units.inchesToMeters(Double.parseDouble(values[0])),
                        Units.inchesToMeters(Double.parseDouble(values[1])),
                        Rotation2d.fromDegrees(Double.parseDouble(values[2])));

            } 
            else { // Ignores rotation for inner points
                interiorPoints.add(
                        new Translation2d(
                                Units.inchesToMeters(Double.parseDouble(values[0])),
                                Units.inchesToMeters(Double.parseDouble(values[1]))));
            }

        }
        sc.close();

    }

    public Trajectory getTrajectory(TrajectoryConfig config) {

        // SmartDashboard.putString("Testing/List", interiorPoints.toString());

        // Generate tajectory with passed in config
        Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
                start,
                interiorPoints,
                end,
                config);

        // SmartDashboard.putString("Testing/Trajectory", trajectory.toString());
        return trajectory;
    }
}
