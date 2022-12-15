package org.team1619.models.inputs.vector;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.LimitedSizeQueue;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.purepursuit.Pose2d;
import org.uacr.utilities.purepursuit.Vector;
import org.uacr.utilities.Timer;

import java.util.HashMap;
import java.util.Map;
import java.util.Queue;

/**
 * LimelightOdometry is an input vector which uses input from the Limelight and IMU to
 * determine robot position
 *
 * @author Harrison Kaufman
 */

public class LimelightOdometry extends BaseOdometry {
    private static final Logger logger = LogManager.getLogger(LimelightOdometry.class);

    private String limelight;
    private String imu;
    private Queue<Double> limelightTxValues;
    private boolean isValid;
    private Timer updateTimer;
    private int updateTime;
    private double maxAngleChange;
    private int maxAngleChangeFrames;
    private int frameCount;
    private double previousAngle;

    public LimelightOdometry(Object name, Config config, InputValues inputValues) {
        super(name, config, inputValues, UpdateMode.ABSOLUTE_POSITION);
        limelight = config.getString("limelight");
        imu = config.getString("imu");
        limelightTxValues = new LimitedSizeQueue<>(30);
        updateTimer = new Timer();
        updateTime = config.getInt("update_time", -1);
        maxAngleChange = config.getDouble("max_angle_change", 100.0);
        maxAngleChangeFrames = config.getInt("max_angle_change_frames", 0);
        previousAngle = 0;
        frameCount = 0;
    }

    @Override
    public void initialize() {
        isValid = false;
        updateTimer.start(0);
    }

    @Override
    protected Pose2d getPositionUpdate() {
        // Calculate the current x, y location on the field using the limelight
        // Note that when the limelight does not have a target, tv = 0 and distance is -1.0
        // This will cause the calculations to be ignored in the odometry fuser as valid will be set to 0.0

        // Heading:
        // 0 = pointed towards opponents driver station wall
        // 90 = pointed towards the left side
        // 180 or -180 = pointed towards our driver station wall
        // -90 or 270 = pointed to the right side
        // Rotation counter clockwise is +, clockwise is -

        // X,Y graph:
        // +x is the far side of the field
        // -x is the close side of the field
        // + y is the left side of the field
        // - y is the right side of the field

        // Limelight:
        // tx is positive when the target is to the right of center, negative when it is to the left
        // ty is positive when the target is above center, negative when below center

        Vector currentPosition = new Vector();
        double odometryHeading = sharedInputValues.getVector(imu).getOrDefault("heading", 0.0);
        double distance = sharedInputValues.getNumeric("ipn_limelight_distance_to_center_inches");
        double tv = sharedInputValues.getVector(limelight).getOrDefault("tv", 0.0);
        double tx = sharedInputValues.getVector(limelight).getOrDefault("tx", 0.0);

        isValid = false;
        boolean limelightOn = sharedInputValues.getBoolean("ipb_limelight_prime_on") || sharedInputValues.getBoolean("ipb_limelight_on");

        // Only compute if the limelight has a lock
        if (tv > 0 && limelightOn) {

            // Only valid if the imu's change in angle is less than a maximum for a set number of frames
            // This is to prevent a miscalculation caused by the imu updating faster than the limelight when rotating quickly
            double deltaAngle = Math.abs(odometryHeading - previousAngle);
            if(deltaAngle < maxAngleChange){
                frameCount++;
            } else {
                frameCount = 0;
                logger.error("reset frame count");
            }

            // Only compute if we have a valid distance
            if((distance >= 0) && (frameCount > maxAngleChangeFrames)) {
                isValid = true;
                currentPosition = new Vector(distance, (odometryHeading - tx));
            }
        } else {
            frameCount = 0;
        }
        previousAngle = odometryHeading;

        return new Pose2d(-currentPosition.getX(), -currentPosition.getY(), odometryHeading);
    }

    @Override
    public Map<String, Double> get() {
        Map<String, Double> data = new HashMap<>(super.get());

        data.put("valid", (isValid && (updateTime <= 0 || updateTimer.isDone())) ? 1.0 : 0.0);

        if(isValid && updateTime > 0 && updateTimer.isDone()) {
            updateTimer.start(updateTime);
        }
        return data;
    }

    @Override
    protected void zero() {
    }

}