package org.team1619.models.inputs.vector;

import org.uacr.models.inputs.vector.InputVector;
import org.uacr.utilities.Config;

import java.util.HashMap;
import java.util.Map;

public abstract class Pigeon extends InputVector {

    protected final int deviceNumber;
    protected Map<String, Boolean> isRaidans;
    protected Map<String, Boolean> isInverted;
    protected Map<String, Double> pigeonValues;
    protected final int Status4CANbusUpdatePeriod;
    protected final int Status6CANbusUpdatePeriod;
    protected final int Status9CANbusUpdatePeriod;
    protected final int Status11CANbusUpdatePeriod;

    public Pigeon(Object name, Config config) {
        super(name, config);

        deviceNumber = config.getInt("device_number");

        pigeonValues = new HashMap<>();

        //Is Inverted
        isInverted = new HashMap<>();
        isInverted.put("yaw", config.getBoolean("yaw_is_inverted", false));
        isInverted.put("roll", config.getBoolean("roll_is_inverted", false));
        isInverted.put("pitch", config.getBoolean("pitch_is_inverted", false));
        isInverted.put("compass", config.getBoolean("compass_is_inverted", false));
        isInverted.put("angle", config.getBoolean("angle_is_inverted", false));
        isInverted.put("accel_x", config.getBoolean("accel_x_is_inverted", false));
        isInverted.put("accel_y", config.getBoolean("accel_y_is_inverted", false));
        isInverted.put("accel_z", config.getBoolean("accel_z_is_inverted", false));
        isInverted.put("trig_angle", config.getBoolean("trig_angle_is_inverted", false));
        isInverted.put("trig_angle_acc", config.getBoolean("trig_angle_acc_is_inverted", false));

        // Is radians
        isRaidans = new HashMap<>();
        isRaidans.put("yaw", config.getBoolean("yaw_is_radians", false));
        isRaidans.put("roll", config.getBoolean("roll_is_radians", false));
        isRaidans.put("pitch", config.getBoolean("pitch_is_radians", false));
        isRaidans.put("compass", config.getBoolean("compass_is_radians", false));
        isRaidans.put("angle", config.getBoolean("angle_is_radians", false));
        isRaidans.put("trig_angle", config.getBoolean("trig_angle_is_radians", false));
        isRaidans.put("trig_angle_acc", config.getBoolean("trig_angle_acc_is_radians", false));

        // Sets the time between messages for specific message types on the CAN bus
        // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html#pigeon-imu
        Status4CANbusUpdatePeriod = config.getInt("magnetometer_can_bus_update_period", 20);
        Status6CANbusUpdatePeriod = config.getInt("sensor_fusion_can_bus_update_period", 10);
        Status9CANbusUpdatePeriod = config.getInt("yaw_pitch_roll_can_bus_update_period", 10);
        Status11CANbusUpdatePeriod = config.getInt("gyro_can_bus_update_period", 20);
    }

    @Override
    public void update() {
        pigeonValues = readHardware();
    }

    @Override
    public void initialize() {
        pigeonValues =  new HashMap<>() {{
            put("yaw", 0.0);
            put("roll", 0.0);
            put("pitch", 0.0);
            put("compass", 0.0);
            put("angle", 0.0);
            put("accel_x", 0.0);
            put("accel_y", 0.0);
            put("accel_z", 0.0);
            put("trig_angle", 0.0);
            put("trig_angle_acc", 0.0);
        }};
    }

    @Override
    public Map<String, Double> get() {
        return pigeonValues;
    }

    public void processFlag(String flag) {
        if (flag.equals("zero")) {
            zeroYaw();
        }
    }


    protected abstract Map<String, Double> readHardware();

    protected abstract void zeroYaw();
}
