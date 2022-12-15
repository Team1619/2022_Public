package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.Point;
import org.uacr.utilities.purepursuit.Vector;

import java.util.Map;

/**
 * Drives the robot in swerve mode, based on the joystick values
 */

public class Drivetrain_Swerve extends BaseSwerve {

    private static final Logger logger = LogManager.getLogger(Drivetrain_Swerve.class);

    private final String xAxis;
    private final String yAxis;
    private final String rotateAxis;
    private final String fieldOrientedButton;
    private final String zeroAngleButton;
    private final String slowModeButton;
    private final String headingAngleAdjustButton;

    private final double slowModeMaxVelocity;
    private final double joystickExponent;
    private final double deadzone;
    private final double headingAdjustAmount;

    private final int primedToShootFrames;

    private String alignSource;

    private double angle;
    private double alignmentThreshold;

    private int primedToShootCount;
    private int previousAdjustHeadingAngle;

    private boolean isPrimedToShoot;
    private boolean fieldOriented;
    private boolean isClimbing;

    public Drivetrain_Swerve(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
        super(inputValues, outputValues, robotConfiguration, true);

        xAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_x");
        yAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_y");
        rotateAxis = robotConfiguration.getString("global_drivetrain_swerve", "swerve_rotate");
        fieldOrientedButton = robotConfiguration.getString("global_drivetrain_swerve", "swerve_field_oriented_button");
        zeroAngleButton = robotConfiguration.getString("global_drivetrain_swerve", "angle_zero_button");
        slowModeButton = robotConfiguration.getString("global_drivetrain_swerve", "slow_mode_button");
        headingAngleAdjustButton = robotConfiguration.getString("global_drivetrain_swerve", "heading_angle_adjust_button");

        slowModeMaxVelocity = robotConfiguration.getInt("global_drivetrain_swerve", "slow_mode_max_velocity");
        joystickExponent = robotConfiguration.getDouble("global_drivetrain_swerve", "joystick_exponent");
        deadzone = robotConfiguration.getDouble("global_drivetrain_swerve", "joystick_deadzone");
        headingAdjustAmount = robotConfiguration.getDouble("global_drivetrain_swerve", "heading_angle_adjust_amount");

        primedToShootFrames = robotConfiguration.getInt("global_targeting", "primed_to_shoot_frames");

        angle = 0.0;

        previousAdjustHeadingAngle = 0;

        fieldOriented = true;
        isClimbing = false;
    }

    @Override
    public void initialize(String stateName, Config config) {
        logger.debug("Entering state {}", stateName);

        sharedInputValues.setBoolean("ipb_field_oriented", fieldOriented);
        sharedInputValues.setBoolean("ipb_drivetrain_primed", false);

        alignSource = config.getString("align_source", "none");
        sharedInputValues.setString("ips_alignment_mode", alignSource);

        alignmentThreshold = config.getDouble("alignment_threshold", 1.0);
        angle = config.getDouble("angle", 0.0);

        primedToShootCount = 0;

        isPrimedToShoot = false;
        isClimbing = config.getBoolean("climbing", false);

        stopModules();
    }

    @Override
    public void update() {
        if (sharedInputValues.getBooleanRisingEdge(fieldOrientedButton)) {
            fieldOriented = !fieldOriented;
            sharedInputValues.setBoolean("ipb_field_oriented", fieldOriented);
        }

        if (sharedInputValues.getBooleanRisingEdge(zeroAngleButton)) {
            sharedInputValues.setInputFlag(imu, "zero");
            sharedInputValues.setVector(odometry + "_new_position", Map.of("x", 0.0, "y", 0.0, "heading", 0.0));
        }

        int adjustHeadingAngle = (int) sharedInputValues.getNumeric(this.headingAngleAdjustButton);

        if (adjustHeadingAngle != previousAdjustHeadingAngle) {
            previousAdjustHeadingAngle = adjustHeadingAngle;
            this.headingOffset += headingAdjustAmount * adjustHeadingAngle;
        }

        sharedInputValues.setNumeric("ipn_drivetrain_heading_angle_offset", headingOffset);
        sharedInputValues.setNumeric("ipn_joystick_exponent", joystickExponent);

        boolean slowModeButton = sharedInputValues.getBoolean(this.slowModeButton);
        double xAxis = sharedInputValues.getNumeric(this.xAxis);
        double yAxis = sharedInputValues.getNumeric(this.yAxis);
        double rotateAxis = sharedInputValues.getNumeric(this.rotateAxis);

        xAxis = linearize(xAxis, yAxis);
        yAxis = linearize(yAxis, xAxis);
        rotateAxis = rangeStick(rotateAxis);

        if (slowModeButton) {
            currentMaxModuleVelocity = slowModeMaxVelocity;
        } else {
            currentMaxModuleVelocity = maxModuleVelocity;
            xAxis = exponentialize(xAxis, joystickExponent);
            yAxis = exponentialize(yAxis, joystickExponent);
            rotateAxis = exponentialize(rotateAxis, joystickExponent);
        }

        sharedInputValues.setNumeric("ipn_drivetrain_max_velocity", currentMaxModuleVelocity);

        // This is the orientation of the front of the robot based on the unit circle. It does not have to be 0.
        double robotOrientation = 0;

        // When using field orientation, forward is always towards the opposite end of the field even if the robot is facing a different direction.
        // To do this, the angle of the robot read from the imu is subtracted from the direction chosen by the driver.
        // For example, if the robot is rotated 15 degrees and the driver chooses straight forward, the actual angle is -15 degrees.
        if (fieldOriented) {
            robotOrientation -= sharedInputValues.getVector(odometry).get("heading_acc");
        }

        // Swapping X and Y translates coordinate systems from the controller to the robot.
        // The controller use the Y axis for forward/backwards and the X axis for right/left
        // The robot forward/backwards is along the X axis and left/right is along the Y axis
        Vector translation = new Vector(new Point(yAxis, xAxis)).rotate(robotOrientation);

        // Use the limelight to rotate to aim at the goal
        boolean isPrimed;
        double heading;

        switch (alignSource) {
            case "limelight":
                // Use limelight to point towards the goal
                setModulePowers(translation, "limelight", 0, "pr_align");
                isPrimed = (Math.abs(sharedInputValues.getVector(limelight).get("tx")) + this.headingOffset) < alignmentThreshold;
                break;
            case "fused_odometry":
                // Use odometry to point towards the goal
                setModulePowers(translation, "fused_odometry", 0, "pr_align");
                isPrimed = Math.abs((sharedInputValues.getNumeric("ipn_fused_odometry_delta_angle")) + this.headingOffset) < alignmentThreshold;
                break;
            case "imu":
                // Use angle to point towards the goal
                setModulePowers(translation, "imu", angle + this.headingOffset, "pr_align");
                heading = sharedInputValues.getVector(odometry).getOrDefault("heading", 0.0);
                isPrimed = Math.abs(heading - (angle + this.headingOffset)) < alignmentThreshold;
                break;
            case "angle":
                // Use angle to point towards the goal
                setModulePowers(translation, "angle", angle + this.headingOffset, "pr_align");
                heading = sharedInputValues.getVector(odometry).getOrDefault("heading", 0.0);
                isPrimed = Math.abs(heading - (angle + this.headingOffset)) < alignmentThreshold;
                break;
            default:
                // Driver manually points towards the goal
                setModulePowers(translation, rotateAxis, "pr_drive");
                isPrimed = false;
                break;
        }

        // Require at least 10 prime to shoot frames in a row
        if (isPrimed) {
            primedToShootCount++;
        } else {
            primedToShootCount = 0;
        }

        isPrimedToShoot = primedToShootCount > primedToShootFrames;

        sharedInputValues.setBoolean("ipb_drivetrain_primed", isPrimedToShoot);

        if (isClimbing) {
            angleOutputNames.forEach(n -> sharedOutputValues.setNumeric(n, "absolute_position", 270, "pr_drive"));
        }
    }

    @Override
    public void dispose() {
        stopModules();
    }

    @Override
    public boolean isDone() {
        if (alignSource.equals("limelight") || alignSource.equals("fused_odometry")) {
            return isPrimedToShoot;
        }

        return true;
    }

    private double rangeStick(double value) {
        double valueSign = Math.signum(value);
        value = Math.abs(value);

        double min = deadzone;
        double max = 1.0;

        if (value < min) {
            return 0.0;
        }

        double minOutput = 0.4;
        double maxOutput = 1.0;

        return (((value - min) / (max - min)) * (maxOutput - minOutput) + minOutput) * valueSign;
    }

    private double linearize(double value1, double value2) {
        double magnitude = Math.sqrt(value1 * value1 + value2 * value2);

        double min = deadzone;
        double max = 1.0;

        if (magnitude < min) {
            return 0.0;
        }

        double minOutput = 0.4;
        double maxOutput = 1.0;

        double linearMagnitude = (((magnitude - min) / (max - min)) * (maxOutput - minOutput) + minOutput);

        double multiplier = linearMagnitude / magnitude;

        return value1 * multiplier;
    }

    /**
     * If given a negative value, preserve the sign and raise the magnitude to the appropriate power
     * If given a positive value, just raise it to the appropriate power
     *
     * @param raw   the raw joystick value
     * @param power the desired exponent
     * @return the exponentialized value
     **/

    public double exponentialize(double raw, double power) {
        double sign = Math.signum(raw);
        double magnitude = Math.abs(raw);

        return sign * Math.pow(magnitude, power);
    }
}
