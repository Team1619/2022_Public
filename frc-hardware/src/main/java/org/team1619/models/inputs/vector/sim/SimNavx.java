package org.team1619.models.inputs.vector.sim;

import org.team1619.models.inputs.vector.Navx;
import org.uacr.shared.abstractions.EventBus;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.HashMap;
import java.util.Map;

public class SimNavx extends Navx {

    private static final Logger logger = LogManager.getLogger(SimNavx.class);

    private final SimInputVectorListener listener;

    private double navxYaw;
    private double navxRoll;
    private double navxPitch;
    private double navxCompass;
    private double navxAngle;
    private double navxFusedHeading;
    private double navxAccelX;
    private double navxAccelY;
    private double navxAccelZ;
    private double trigAngle;
    private double trigAngleAcc;


    public SimNavx(EventBus eventBus, Object name, Config config) {
        super(name, config);
        listener = new SimInputVectorListener(eventBus, name, valueMap());
    }

    protected Map<String, Double> readHardware() {

        //Inverted
        navxYaw = getValue("yaw");
        navxRoll = getValue("roll");
        navxPitch = getValue("pitch");
        navxCompass = getValue("compass");
        navxAngle = getValue("angle");
        navxFusedHeading = getValue("fused_heading");
        navxAccelX = getValue("accel_x");
        navxAccelY = getValue("accel_y");
        navxAccelZ = getValue("accel_z");

        // Create a consistant IMU output that shows the angle of the robot based on trig from 0 to 360
        trigAngleAcc = -navxAngle;
        trigAngle = (trigAngleAcc % 360.0);
        trigAngle = (trigAngle < 0) ? (trigAngle + 360) : trigAngle;

        return valueMap();
    }

    private Map<String, Double> valueMap() {
        return new HashMap<>() {{
            put("yaw", navxYaw);
            put("roll", navxRoll);
            put("pitch", navxPitch);
            put("compass", navxCompass);
            put("angle", navxAngle);
            put("fused_heading", navxFusedHeading);
            put("accel_x", navxAccelX);
            put("accel_y", navxAccelY);
            put("accel_z", navxAccelZ);
            put("trig_angle", trigAngle);
            put("trig_angle_acc", trigAngleAcc);
        }};
    }

    private double getValue(String name) {
        double value = isInverted.get(name) ? listener.get().get(name) * -1 : listener.get().get(name);
        return (isRadians.containsKey(name) && isRadians.get(name)) ? value * Math.PI / 180 : value;
    }

    protected void zeroYaw() {
        logger.debug("SimNavxInput -> Zeroing yaw");
        double yaw = navxValues.get("yaw");

        Map<String, Double> lastNavxValues = navxValues;
        navxValues = new HashMap<>();
        navxValues.putAll(lastNavxValues);

        navxValues.put("yaw", 0.0);
    }
}
