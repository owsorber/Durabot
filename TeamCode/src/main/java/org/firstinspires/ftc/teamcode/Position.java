package org.firstinspires.ftc.teamcode;

/**
 * The Position class which stores a <x, y> vector. Part of the instance data for the Robot.
 *
 * @author Ben Gerrard
 * @date May 24th, 2018
 */

public class Position {
    private double x;
    private double y;

    // Default constructor
    public Position() {
        x = 0;
        y = 0;
    }

    // Constructor that takes in x and y
    public Position(double xVal, double yVal) {
        x = xVal;
        y = yVal;
    }

    // X and Y accessor methods
    public double getX() {
        return x;
    }
    public double getY() {
        return y;
    }

    public void setX(int xVal) {
        x = xVal;
    }
    public void setY(int yVal) {
        y = yVal;
    }

    // Add another Position to this Position
    public void add(Position other) {
        x += other.getX();
        y += other.getY();
    }

    // Returns a Position object that is the change in position from this position object to another position object
    public Position changeInPosition(Position other) {
        double newX = other.getX() - x;
        double newY = other.getY() - y;

        return new Position(newX, newY);
    }

    // Return Position as a String, used to outputting Position to telemetry
    public String toString() {
        return "(" + (int) x + ", " + (int) y + ")";
    }

    // Calculates Position's distance from another position
    public double distanceFrom(Position other) {
        double xSquared = Math.pow(x - other.getX(), 2.0), ySquared = Math.pow(y - other.getY(), 2.0);

        return Math.sqrt(xSquared + ySquared);
    }

    // Calculates Position's distance from origin (vector magnitude)
    public double distFromOrigin() {
        return distanceFrom(new Position());
    }

    // Return quadrant of position on cartesian plane (if in no quadrant, return 0)
    public int getQuadrant() {
        if (x > 0 && y > 0) return 1;
        else if (x < 0 && y < 0) return 3;
        else if (x < 0 && y > 0) return 2;
        else if (x > 0 && y < 0) return 4;
        return 0;
    }
}
