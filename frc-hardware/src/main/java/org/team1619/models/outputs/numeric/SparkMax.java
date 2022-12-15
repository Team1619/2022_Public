package org.team1619.models.outputs.numeric;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.utilities.Config;

public abstract class SparkMax extends REVMotor {

    protected final InputValues sharedInputValues;

    public SparkMax(Object name, Config config, InputValues inputValues) {
        super(name, config);

        sharedInputValues = inputValues;
    }
}
