package org.uacr.utilities;

public final class Util {

    private Util() {

    }

    /**
     * Limits range of value with maxMagnitude
     *
     * @param value        the initial double
     * @param maxMagnitude the max distance from 0 for the bounded value
     * @return the bounded number
     */
    public static double limit(double value, double maxMagnitude) {
        return limit(value, -maxMagnitude, maxMagnitude);
    }

    /**
     * Limits range of value with min and max
     *
     * @param value the initial double
     * @param min   the lower bound of the return value
     * @param max   the upper bound of the return value
     * @return the bounded number
     */
    public static double limit(double value, double min, double max) {
        return Math.min(Math.max(value, min), max);
    }

    /**
     * Returns whether the value is within maxMagnitude of 0 (inclusive)
     *
     * @param value        the double to compare
     * @param maxMagnitude the max distance from 0
     * @return whether the value is in the range
     */
    public static boolean inRange(double value, double maxMagnitude) {
        return inRange(value, -maxMagnitude, maxMagnitude);
    }

    /**
     * Returns whether the value is within the range of min and max (inclusive)
     *
     * @param value the double to compare
     * @param min   the lower bound of the range
     * @param max   the upper bound of the range
     * @return whether the value is in the range
     */
    public static boolean inRange(double value, double min, double max) {
        return min <= value && value <= max;
    }

    /**
     * Returns initial value or 0 if the initial value is less than minMagnitude from 0
     *
     * @param value        the double to deadband
     * @param minMagnitude the min distance from 0 for the value
     * @return whether the value is in the range
     */
    public static double deadband(double value, double minMagnitude) {
        if (-minMagnitude < value && value < minMagnitude) {
            return 0.0;
        }
        return value;
    }

    public static double min(double... values) {
        double minValue = Double.MAX_VALUE;
        for (double value : values) {
            minValue = Math.min(minValue, value);
        }
        return minValue;
    }

    public static double max(double... values) {
        double maxValue = Double.MIN_VALUE;
        for (double value : values) {
            maxValue = Math.max(maxValue, value);
        }
        return maxValue;
    }

    public static double interpolate(double input, double minInput, double maxInput, double minOutput, double maxOutput) {
        return minOutput + ((maxOutput - minOutput) * ((limit(input, minInput, maxInput) - minInput) / (maxInput - minInput)));
    }
}
