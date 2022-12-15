package org.team1619.models.outputs.numeric;

import org.uacr.models.outputs.numeric.OutputNumeric;
import org.uacr.utilities.Config;

/**
 * Controls the Spike relay via the RoboRio Relay ports 0 through 3.
 */

public abstract class RoboRioRelay extends OutputNumeric {

    protected final int deviceNumber;

    public RoboRioRelay(Object name, Config config) {
        super(name, config);

        deviceNumber = config.getInt("device_number");
    }
}
