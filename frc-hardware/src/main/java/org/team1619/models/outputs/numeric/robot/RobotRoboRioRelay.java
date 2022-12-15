package org.team1619.models.outputs.numeric.robot;

import edu.wpi.first.wpilibj.Relay;
import org.team1619.models.outputs.numeric.RoboRioRelay;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.utilities.Config;

public class RobotRoboRioRelay extends RoboRioRelay {
    private final Relay wpiRelay;

    public RobotRoboRioRelay(Object name, Config config, HardwareFactory hardwareFactory) {
        super(name, config);

        wpiRelay = hardwareFactory.get(Relay.class, deviceNumber);
        wpiRelay.set(Relay.Value.kOff);
    }

    @Override
    public void processFlag(String flag) {

    }

    @Override
    public void setHardware(String outputType, double outputValue, String profile) {
        if (outputValue == -1) {
            wpiRelay.set(Relay.Value.kReverse);
        }
        else if (outputValue == 0) {
            wpiRelay.set(Relay.Value.kOff);
        }
        else if (outputValue == 1) {
            wpiRelay.set(Relay.Value.kForward);
        }
    }
}
