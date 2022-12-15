package org.team1619.models.outputs.bool.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import org.team1619.models.outputs.bool.SolenoidDouble;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;

public class RobotSolenoidDouble extends SolenoidDouble {

    private final DoubleSolenoid wpiSolenoid;

    public RobotSolenoidDouble(Object name, Config config, HardwareFactory hardwareFactory, RobotConfiguration robotConfiguration) {
        super(name, config);
        int deviceId = robotConfiguration.getInt("global_pneumatics", "device_id");
        Enum<PneumaticsModuleType> moduleType = robotConfiguration.getEnum("global_pneumatics", "module_type", PneumaticsModuleType.class);
        wpiSolenoid = hardwareFactory.get(DoubleSolenoid.class, deviceId, moduleType, deviceNumberPrimary, deviceNumberFollower);
        wpiSolenoid.set(DoubleSolenoid.Value.kOff);
    }

    @Override
    public void processFlag(String flag) {

    }

    @Override
    public void setHardware(boolean output) {
        if (output) {
            wpiSolenoid.set(DoubleSolenoid.Value.kForward);
        } else {
            wpiSolenoid.set(DoubleSolenoid.Value.kReverse);
        }
    }
}