package org.team1619.models.outputs.bool.robot;

import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Solenoid;
import org.team1619.models.outputs.bool.SolenoidSingle;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;

public class RobotSolenoidSingle extends SolenoidSingle {

    private final Solenoid wpiSolenoid;

    public RobotSolenoidSingle(Object name, Config config, HardwareFactory hardwareFactory, RobotConfiguration robotConfiguration) {
        super(name, config);

        int deviceId = robotConfiguration.getInt("global_pneumatics", "device_id");
        Enum<PneumaticsModuleType> moduleType = robotConfiguration.getEnum("global_pneumatics", "module_type", PneumaticsModuleType.class);
        wpiSolenoid = hardwareFactory.get(Solenoid.class, deviceId, moduleType, deviceNumber);
    }

    @Override
    public void processFlag(String flag) {

    }

    @Override
    public void setHardware(boolean output) {
        wpiSolenoid.set(output);
    }
}