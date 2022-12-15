package org.team1619.models.inputs.vector;

import org.uacr.models.inputs.vector.InputVector;
import org.uacr.utilities.Config;

import java.util.HashMap;
import java.util.Map;

public abstract class Navx extends InputVector {

    protected Map<String, Boolean> isRadians;
    protected Map<String, Boolean> isInverted;
    protected Map<String, Double> navxValues;

    public Navx(Object name, Config config) {
        super(name, config);

        navxValues = new HashMap<>();

        //Is Inverted
        isInverted = new HashMap<>();
        isInverted.put("yaw", config.getBoolean("yaw_is_inverted", false));
        isInverted.put("roll", config.getBoolean("roll_is_inverted", false));
        isInverted.put("pitch", config.getBoolean("pitch_is_inverted", false));
        isInverted.put("compass", config.getBoolean("compass_is_inverted", false));
        isInverted.put("angle", config.getBoolean("angle_is_inverted", false));
        isInverted.put("fused_heading", config.getBoolean("fused_heading_is_inverted", false));
        isInverted.put("accel_x", config.getBoolean("accel_x_is_inverted", false));
        isInverted.put("accel_y", config.getBoolean("accel_y_is_inverted", false));
        isInverted.put("accel_z", config.getBoolean("accel_z_is_inverted", false));
        isInverted.put("trig_angle", config.getBoolean("trig_angle_is_inverted", false));
        isInverted.put("trig_angle_acc", config.getBoolean("trig_angle_acc_is_inverted", false));

        // Is radians
        isRadians = new HashMap<>();
        isRadians.put("yaw", config.getBoolean("yaw_is_radians", false));
        isRadians.put("roll", config.getBoolean("roll_is_radians", false));
        isRadians.put("pitch", config.getBoolean("pitch_is_radians", false));
        isRadians.put("compass", config.getBoolean("compass_is_radians", false));
        isRadians.put("angle", config.getBoolean("angle_is_radians", false));
        isRadians.put("trig_angle", config.getBoolean("trig_angle_is_radians", false));
        isRadians.put("trig_angle_acc", config.getBoolean("trig_angle_acc_is_radians", false));
        isRadians.put("fused_heading", config.getBoolean("fused_heading_is_radians", false));
    }

    @Override
    public void update() {
        navxValues = readHardware();
    }

    @Override
    public void initialize() {
        navxValues = new HashMap<>(){{
            put("yaw", 0.0);
            put("roll", 0.0);
            put("pitch", 0.0);
            put("compass", 0.0);
            put("angle", 0.0);
            put("fused_heading", 0.0);
            put("accel_x", 0.0);
            put("accel_y", 0.0);
            put("accel_z", 0.0);
            put("trig_angle", 0.0);
            put("trig_angle_acc", 0.0);
        }};
    }

    @Override
    public Map<String, Double> get() {
        return navxValues;
    }

    public void processFlag(String flag) {
        if (flag.equals("zero")) {
            zeroYaw();
        }
    }


    protected abstract Map<String, Double> readHardware();

    protected abstract void zeroYaw();
}
