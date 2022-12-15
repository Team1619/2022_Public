package org.team1619.models.inputs.vector;

import org.uacr.models.inputs.vector.InputVector;
import org.uacr.utilities.Config;

import java.util.HashMap;
import java.util.Map;

public abstract class ColorSensor extends InputVector {

    protected  Map<String, Double> colorValues;

    public ColorSensor(Object name, Config config) {
        super(name, config);
        colorValues = new HashMap<>();
    }

    @Override
    public void initialize() {
    }

    @Override
    public void update() {
        colorValues = getValue();
    }

    @Override
    public Map<String, Double> get() {
        return colorValues;
    }

    public abstract Map<String, Double> getValue();

    @Override
    public void processFlag(String flag) {}
}
