package org.team1619.models.outputs.numeric;

import org.uacr.models.outputs.numeric.OutputNumeric;
import org.uacr.utilities.Config;

public abstract class REVMotor extends OutputNumeric {

    protected final boolean isBrakeModeEnabled;

    protected final int deviceNumber;

    public REVMotor(Object name, Config config) {
        super(name, config);

        isBrakeModeEnabled = config.getBoolean("brake_mode_enabled", true);

        deviceNumber = config.getInt("device_number");
    }

    public int getDeviceNumber() {
        return deviceNumber;
    }
}
