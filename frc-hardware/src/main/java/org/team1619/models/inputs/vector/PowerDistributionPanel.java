package org.team1619.models.inputs.vector;

import edu.wpi.first.wpilibj.PowerDistribution;
import org.uacr.models.inputs.vector.InputVector;
import org.uacr.utilities.Config;

import java.util.Map;

public abstract class PowerDistributionPanel extends InputVector {

    protected Map<String, Double> pdpReadings;
    protected final PowerDistribution.ModuleType moduleType;
    protected final int deviceNumber;

    public PowerDistributionPanel(Object name, Config config) {
        super(name, config);
        moduleType = config.getEnum("module_type", PowerDistribution.ModuleType.class);
        deviceNumber = config.getInt("device_number");
    }

    @Override
    public void initialize() {

    }

    @Override
    public void update() {
        pdpReadings = readHardware();
    }

    @Override
    public Map<String, Double> get() {
        return pdpReadings;
    }

    @Override
    public void processFlag(String s) {

    }

    protected abstract Map<String, Double> readHardware();
}
