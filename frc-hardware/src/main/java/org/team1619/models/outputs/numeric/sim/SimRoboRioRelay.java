package org.team1619.models.outputs.numeric.sim;

import edu.wpi.first.wpilibj.Relay;
import org.team1619.models.outputs.numeric.RoboRioRelay;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;

public class SimRoboRioRelay extends RoboRioRelay {

    public SimRoboRioRelay(Object name, Config config) {
        super(name, config);
    }

    @Override
    public void processFlag(String flag) {

    }

    @Override
    public void setHardware(String outputType, double outputValue, String profile) {

    }
}
