package org.team1619.models.inputs.vector.sim;

import org.team1619.models.inputs.vector.Pigeon;
import org.uacr.shared.abstractions.EventBus;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.HashMap;
import java.util.Map;

public class SimPigeon extends Pigeon {

    private static final Logger sLogger = LogManager.getLogger(SimPigeon.class);

    private final SimInputVectorListener fListener;

    private double pigeonYaw;
    private double pigeonRoll;
    private double pigeonPitch;
    private double pigeonCompass;
    private double pigeonAccelX;
    private double pigeonAccelY;
    private double pigeonAccelZ;
    private double pigeonAngle;
    private double trigAngle;
    private double trigAngleAcc;


    public SimPigeon(EventBus eventBus, Object name, Config config) {
        super(name, config);

        fListener = new SimInputVectorListener(eventBus, name, valueMap());
    }

    protected Map<String, Double> readHardware() {

        //Inverted
        pigeonYaw = getValue("yaw");
        pigeonRoll = getValue("roll");
        pigeonPitch = getValue("pitch");
        pigeonCompass = getValue("compass");
        pigeonAngle = getValue("angle");
        pigeonAccelX = getValue("accel_x");
        pigeonAccelY = getValue("accel_y");
        pigeonAccelZ = getValue("accel_z");

        // Create a consistant IMU output that shows the angle of the robot based on trig from 0 to 360
        trigAngleAcc = pigeonYaw;
        trigAngle = (trigAngleAcc % 360.0);
        trigAngle = (trigAngle < 0) ? (trigAngle + 360) : trigAngle;

        return valueMap();
    }

    private double getValue(String name) {
        double value = isInverted.get(name) ? fListener.get().get(name) * -1 : fListener.get().get(name);
        return (isRaidans.containsKey(name) && isRaidans.get(name)) ? value * Math.PI / 180 : value;
    }

    protected void zeroYaw() {
        sLogger.debug("pigeonInput -> Zeroing yaw");
        pigeonYaw = 0.0;
        pigeonRoll = 0.0;
        pigeonPitch = 0.0;
        pigeonCompass = 0.0;
        pigeonAngle = 0.0;
        pigeonAccelX = 0.0;
        pigeonAccelY = 0.0;
        pigeonAccelZ = 0.0;
        trigAngleAcc = 0.0;
        trigAngle = 0.0;

        pigeonValues = valueMap();
    }

    private Map<String, Double> valueMap() {
        return new HashMap<>() {{
            put("yaw", pigeonYaw);
            put("roll", pigeonRoll);
            put("pitch", pigeonPitch);
            put("compass", pigeonCompass);
            put("angle", pigeonAngle);
            put("accel_x", pigeonAccelX);
            put("accel_y", pigeonAccelY);
            put("accel_z", pigeonAccelZ);
            put("trig_angle", trigAngle);
            put("trig_angle_acc", trigAngleAcc);
        }};
    }
}
