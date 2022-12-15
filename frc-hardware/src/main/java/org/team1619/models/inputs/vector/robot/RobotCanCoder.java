package org.team1619.models.inputs.vector.robot;

import com.ctre.phoenix.sensors.*;
import org.team1619.models.inputs.vector.Encoder;
import org.uacr.utilities.Config;
import org.uacr.shared.abstractions.HardwareFactory;

import java.util.HashMap;
import java.util.Map;

public class RobotCanCoder extends Encoder {

    private final CANCoder canCoder;
    private final CANCoderConfiguration canCoderConfiguration;

    public RobotCanCoder(Object name, Config config, HardwareFactory hardwareFactory) {
        super(name, config);
        canCoder = hardwareFactory.get(CANCoder.class, deviceNumber);
        canCoderConfiguration = new CANCoderConfiguration();
    }

    @Override
    public void initialize() {
        canCoderConfiguration.unitString = "degrees";
        canCoderConfiguration.initializationStrategy = bootToAbsolutePosition ? SensorInitializationStrategy.BootToAbsolutePosition : SensorInitializationStrategy.BootToZero;
        canCoderConfiguration.absoluteSensorRange = sensorRange360 ? AbsoluteSensorRange.Unsigned_0_to_360 : AbsoluteSensorRange.Signed_PlusMinus180;
        canCoderConfiguration.sensorDirection = inverted;
        canCoderConfiguration.magnetOffsetDegrees = magnetOffset;
        canCoder.configAllSettings(canCoderConfiguration);
        canCoder.setStatusFramePeriod(CANCoderStatusFrame.SensorData, CANbusUpdatePeriod);
    }

    @Override
    protected Map<String, Double> readHardware() {
        Map<String, Double> canCoderValues = new HashMap<>();

        if(readPosition) {
            canCoderValues.put("position", canCoder.getPosition() * positionScalar);
        }
        if (readAbsolutePosition) {
            canCoderValues.put("absolute_position", canCoder.getAbsolutePosition() * positionScalar);
        }
        if (readVelocity){
            canCoderValues.put("velocity", canCoder.getVelocity() * velocityScalar);
        }
        return canCoderValues;
    }

    @Override
    protected void zeroEncoder() {

    }
}

