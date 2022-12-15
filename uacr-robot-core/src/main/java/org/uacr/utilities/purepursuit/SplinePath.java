package org.uacr.utilities.purepursuit;

import org.uacr.utilities.purepursuit.spline.ParametricSpline;

import java.util.ArrayList;
import java.util.Arrays;

/**
 * Class to represent a path that is created using splines.
 */
public class SplinePath extends Path {

    // isNaturalSpline refers to whether it is a "natural spline" (2nd derivative is zero at ends)
    private final boolean isNaturalSpline;
    // If it is not a naturalSpline, the starting and ending angles should be specified
    private Double startingAngle = null;
    private Double endingAngle = null;

    /**
     * Pass in an ArrayList of waypoints
     */
    public SplinePath(ArrayList<Pose2d> waypoints) {
        super(waypoints);
        isNaturalSpline = true;
    }

    /**
     * Pass in a comma separated list or array of waypoints
     */
    public SplinePath(Pose2d... waypoints) {
        this(new ArrayList<>(Arrays.asList(waypoints)));
    }

    public SplinePath(Double startingAngle, Double endingAngle, ArrayList<Pose2d> waypoints) {
        super(waypoints);
        this.startingAngle = startingAngle;
        this.endingAngle = endingAngle;
        isNaturalSpline = false;
    }

    public SplinePath(Double startingAngle, Double endingAngle, Pose2d... waypoints) {
        this(startingAngle, endingAngle, new ArrayList<>(Arrays.asList(waypoints)));
    }

    /**
     * Uses a parametric spline to interpolate waypoints and fill the path.
     */
    @Override
    protected void fill() {
        ParametricSpline spline;
        if (isNaturalSpline) {
            spline = new ParametricSpline(waypoints);
        } else {
            spline = new ParametricSpline(waypoints, startingAngle, endingAngle);
        }
        generatedPoints = spline.getPoints(getPointSpacing());
    }

    @Override
    protected void smooth() {
        // No smoothing needed with spline path
    }
}
