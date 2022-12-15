package org.team1619.models.inputs.vector.robot;

import com.ctre.phoenix.sensors.Pigeon2;
import com.ctre.phoenix.sensors.Pigeon2Configuration;
import com.ctre.phoenix.sensors.PigeonIMU_StatusFrame;
import org.team1619.models.inputs.vector.Pigeon;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.HashMap;
import java.util.Map;

public class RobotPigeon extends Pigeon {

    private static final Logger sLogger = LogManager.getLogger(RobotPigeon.class);

    Pigeon2 pigeon = new Pigeon2(deviceNumber);

    private double pigeonYaw;
    private double pigeonRoll;
    private double pigeonPitch;
    private double pigeonCompass;
    private double pigeonAngle;
    private double pigeonAccelX;
    private double pigeonAccelY;
    private double pigeonAccelZ;
    private double trigAngle;
    private double trigAngleAcc;


    public RobotPigeon(Object name, Config config, HardwareFactory hardwareFactory) {
        super(name, config);

        // Sets the time between messages for specific message types on the CAN bus
        // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html#pigeon-imu
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.RawStatus_4_Mag, Status4CANbusUpdatePeriod);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_6_SensorFusion, Status6CANbusUpdatePeriod);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_9_SixDeg_YPR, Status9CANbusUpdatePeriod);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_11_GyroAccum, Status11CANbusUpdatePeriod);

        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_1_General, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_3_GeneralAccel, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.CondStatus_10_SixDeg_Quat, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_2_Gyro, 255);
        pigeon.setStatusFramePeriod(PigeonIMU_StatusFrame.BiasedStatus_6_Accel, 255);
    }

    @Override
    protected void zeroYaw() {
        Pigeon2Configuration config = new Pigeon2Configuration();
        // set mount pose as rolled 90 degrees counter-clockwise
        config.MountPoseYaw = 0;
        config.MountPosePitch = 0;
        config.MountPoseRoll = 0;
        pigeon.configAllSettings(config);

        sLogger.debug("RobotPigeonInput -> Zeroing yaw");
        pigeon.setYaw(0.0);
    }

    @Override
    protected Map<String, Double> readHardware() {

        // Get Values
        double yawpitchroll[] = new double[3];
        pigeon.getYawPitchRoll(yawpitchroll);
        // yaw is the accumlative z axis - can exceed 360
        pigeonYaw = yawpitchroll[0];
        pigeonPitch = yawpitchroll[1];
        pigeonRoll = yawpitchroll[2];
        pigeonCompass = pigeon.getCompassHeading();
        double xyz[] = new double[3];
        pigeonAccelX = xyz[0];
        pigeonAccelY = xyz[1];
        pigeonAccelZ = xyz[2];

        // Inverted
        pigeonYaw = isInverted.get("yaw") ? pigeonYaw * -1 : pigeonYaw;
        pigeonRoll = isInverted.get("roll") ? pigeonRoll * -1 : pigeonRoll;
        pigeonPitch = isInverted.get("pitch") ? pigeonPitch * -1 : pigeonPitch;
        pigeonCompass = isInverted.get("compass") ? 360 - pigeonCompass : pigeonCompass;
        pigeonAngle = isInverted.get("angle") ? pigeonAngle * -1 : pigeonAngle;
        pigeonAccelX = isInverted.get("accel_x") ? pigeonAccelX * -1 : pigeonAccelX;
        pigeonAccelY = isInverted.get("accel_y") ? pigeonAccelY * -1 : pigeonAccelY;
        pigeonAccelZ = isInverted.get("accel_z") ? pigeonAccelZ * -1 : pigeonAccelZ;

        //Radians
        pigeonYaw = isRaidans.get("yaw") ? pigeonYaw * Math.PI / 180 : pigeonYaw;
        pigeonRoll = isRaidans.get("roll") ? pigeonRoll * Math.PI / 180 : pigeonRoll;
        pigeonPitch = isRaidans.get("pitch") ? pigeonPitch * Math.PI / 180 : pigeonPitch;
        pigeonCompass = isRaidans.get("compass") ? pigeonCompass * Math.PI / 180 : pigeonCompass;
        pigeonAngle = isRaidans.get("angle") ? pigeonAngle * Math.PI / 180 : pigeonAngle;

        // Create a consistent IMU output that shows the angle of the robot based on trig from 0 to 360
        trigAngleAcc = pigeonYaw;
        trigAngle = (trigAngleAcc % 360.0);
        trigAngle = (trigAngle < 0) ? (trigAngle + 360) : trigAngle;
        trigAngleAcc = isInverted.get("trig_angle_acc") ? trigAngleAcc * -1 : trigAngleAcc;
        trigAngleAcc = isRaidans.get("trig_angle_acc") ? trigAngleAcc * Math.PI / 180 : trigAngleAcc;
        trigAngle = isInverted.get("trig_angle") ? trigAngle * -1 : trigAngle;
        trigAngle = isRaidans.get("trig_angle") ? trigAngle * Math.PI / 180 : trigAngle;

        return new HashMap<>(){{
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
