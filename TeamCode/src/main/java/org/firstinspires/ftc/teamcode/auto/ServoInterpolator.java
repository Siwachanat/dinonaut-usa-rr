package org.firstinspires.ftc.teamcode.auto;

import java.util.Arrays;
import java.util.Comparator;

public class ServoInterpolator {

    private double[] knownDistances;
    private double[] knownServoPositions;

    public ServoInterpolator(double[] knownDistances, double[] knownServoPositions) {
        if (knownDistances == null || knownServoPositions == null ||
                knownDistances.length != knownServoPositions.length || knownDistances.length < 2) {
            throw new IllegalArgumentException("Invalid input data. Must provide at least two corresponding distance and position values.");
        }
        this.knownDistances = Arrays.copyOf(knownDistances, knownDistances.length);
        this.knownServoPositions = Arrays.copyOf(knownServoPositions, knownServoPositions.length);

        // Sort the data based on distance for easier interpolation
        sortDataByDistance();
    }

    private void sortDataByDistance() {
        Integer[] indices = new Integer[knownDistances.length];
        for (int i = 0; i < indices.length; i++) {
            indices[i] = i;
        }

        Arrays.sort(indices, Comparator.comparingDouble(i -> knownDistances[i]));

        double[] sortedDistances = new double[knownDistances.length];
        double[] sortedPositions = new double[knownServoPositions.length];

        for (int i = 0; i < indices.length; i++) {
            sortedDistances[i] = knownDistances[indices[i]];
            sortedPositions[i] = knownServoPositions[indices[i]];
        }

        this.knownDistances = sortedDistances;
        this.knownServoPositions = sortedPositions;
    }

    public double interpolatePosition(double targetDistance) {
        if (targetDistance < knownDistances[0]) {
            System.out.println("Warning: Target distance is below the lowest known distance. Extrapolating.");
            return extrapolateLinear(knownDistances[0], knownServoPositions[0], knownDistances[1], knownServoPositions[1], targetDistance);
        }
        if (targetDistance > knownDistances[knownDistances.length - 1]) {
            System.out.println("Warning: Target distance is above the highest known distance. Extrapolating.");
            return extrapolateLinear(knownDistances[knownDistances.length - 2], knownServoPositions[knownDistances.length - 2], knownDistances[knownDistances.length - 1], knownServoPositions[knownDistances.length - 1], targetDistance);
        }

        for (int i = 0; i < knownDistances.length - 1; i++) {
            if (targetDistance >= knownDistances[i] && targetDistance <= knownDistances[i + 1]) {
                // Linear interpolation between two known points
                double x1 = knownDistances[i];
                double y1 = knownServoPositions[i];
                double x2 = knownDistances[i + 1];
                double y2 = knownServoPositions[i + 1];

                // Formula for linear interpolation:
                // y = y1 + (x - x1) * (y2 - y1) / (x2 - x1)
                return y1 + (targetDistance - x1) * (y2 - y1) / (x2 - x1);
            }
        }

        // This should ideally not be reached if the input data is valid and sorted
        throw new IllegalArgumentException("Could not find a valid interval for the target distance.");
    }

    private double extrapolateLinear(double x1, double y1, double x2, double y2, double targetX) {
        // Using the slope of the line to extrapolate
        double slope = (y2 - y1) / (x2 - x1);
        return y1 + slope * (targetX - x1);
    }

    public static double getInterpolatedPosition(double distance) {
        // Example usage with your provided data points
        double[] distances = {100,114.9, 119.1, 125.3, 131.4, 137, 144.4, 151.8, 159, 167.2, 175.2, 183.4, 190.5, 195};
        double[] servoPositions = {0.05,0.25, 0.3, 0.35, 0.4, 0.45, 0.5, 0.55, 0.6, 0.65, 0.7, 0.75, 0.8, 0.85};

        ServoInterpolator interpolator = new ServoInterpolator(distances, servoPositions);
        return interpolator.interpolatePosition(distance);
    }

    public static void main(String[] args) {
        if (args.length != 1) {
            System.out.println("Usage: java ServoInterpolator <distance>");
            return;
        }

        try {
            double targetDistance = Double.parseDouble(args[0]);
            double interpolatedPosition = getInterpolatedPosition(targetDistance);
            System.out.println(interpolatedPosition);
        } catch (NumberFormatException e) {
            System.out.println("Error: Invalid distance value provided.");
        }
    }
}