package org.uacr.utilities.purepursuit;

import java.util.Objects;

/**
 * Pose2d is an add on to the Point class,
 * allowing it to also store heading
 *
 * @author Matthew Oates
 */

public class Pose2d extends Point {

    protected final double heading;

    public Pose2d(double x, double y, double heading) {
        super(x, y);

        this.heading = heading;
    }

    public Pose2d(Point point, double heading) {
        this(point.getX(), point.getY(), heading);
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    public double getHeading() {
        return heading;
    }

    public Pose2d clone() {
        return new Pose2d(x, y, heading);
    }

    public Pose2d add(Point point) {
        return new Pose2d(x + point.getX(), y + point.getY(), heading);
    }

    public Pose2d subtract(Point point) {
        return new Pose2d(x - point.getX(), y - point.getY(), heading);
    }

    public Pose2d add(Pose2d pose2d) {
        return new Pose2d(x + pose2d.getX(), y + pose2d.getY(), heading + pose2d.getHeading());
    }

    @Override
    public boolean equals(Object o) {
        if (this == o) return true;
        if (o == null || getClass() != o.getClass()) return false;
        if (!super.equals(o)) return false;
        Pose2d pose2d = (Pose2d) o;
        return Double.compare(pose2d.heading, heading) == 0;
    }

    @Override
    public int hashCode() {
        return Objects.hash(super.hashCode(), heading);
    }

    public String toString() {
        return String.format("(x:%.4f,y:%.4f,h:%.4f)",x, y, heading);
    }
}
