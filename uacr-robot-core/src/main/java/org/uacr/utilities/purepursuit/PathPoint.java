package org.uacr.utilities.purepursuit;

import java.util.Objects;

/**
 * PathPoint is an add on to the Pose2d class,
 * allowing it to store additional values necessary for the pure pursuit Path class
 *
 * @author Matthew Oates
 */

public class PathPoint extends Pose2d {

    private final double distance;
    private final double curvature;

    private double velocity;

    public PathPoint(double x, double y, double heading, double distance, double curvature) {
        super(x, y, heading);

        this.distance = distance;
        this.curvature = curvature;
    }

    public PathPoint(Pose2d point, double distance, double curvature) {
        this(point.getX(), point.getY(), point.getHeading(), distance, curvature);
    }

    public double getDistance() {
        return distance;
    }

    public double getCurvature() {
        return curvature;
    }

    public double getVelocity() {
        return velocity;
    }

    public void setVelocity(double velocity) {
        this.velocity = velocity;
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), distance, curvature, velocity);
    }

    public String toString() {
        return String.format("(x:%.4f,y:%.4f,h:%.4f,d:%.4f,c:%.4f,v:%.4f)",x, y, heading, distance, curvature, velocity);
    }
}
