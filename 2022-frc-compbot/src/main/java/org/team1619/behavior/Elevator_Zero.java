package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

/**
 * Zeroes the elevator subsystem
 */

public class Elevator_Zero implements Behavior {

    private static final Logger logger = LogManager.getLogger(Elevator_Zero.class);
    private static final Set<String> subsystems = Set.of("ss_elevator");

    private final InputValues sharedInputValues;
    private final OutputValues sharedOutputValues;

    public Elevator_Zero(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
        sharedInputValues = inputValues;
        sharedOutputValues = outputValues;
    }

    @Override
    public void initialize(String stateName, Config config) {
        logger.debug("Entering state {}", stateName);

        sharedOutputValues.setNumeric("opn_elevator_front_belt", "percent",0.0);
        sharedOutputValues.setNumeric("opn_elevator_back_belt", "percent",0.0);
        sharedOutputValues.setNumeric("opn_elevator_collector_belt", "percent",0.0);
    }

    @Override
    public void update() {
        if (!sharedInputValues.getBoolean("ipb_elevator_has_been_zeroed")) {
            sharedOutputValues.setNumeric("opn_elevator_front_belt", "percent",0.0);
            sharedOutputValues.setNumeric("opn_elevator_back_belt", "percent",0.0);
            sharedOutputValues.setNumeric("opn_elevator_collector_belt", "percent",0.0);
            sharedOutputValues.setOutputFlag("opn_elevator_front_belt","zero");
            sharedOutputValues.setOutputFlag("opn_elevator_back_belt", "zero");
            sharedOutputValues.setOutputFlag("opn_elevator_collector_belt", "zero");
            sharedInputValues.setBoolean("ipb_elevator_has_been_zeroed", true);
            logger.debug("Elevator Zero -> Zeroed");
        }
    }

    @Override
    public void dispose() {
    }

    @Override
    public boolean isDone() {
        return sharedInputValues.getBoolean("ipb_elevator_has_been_zeroed");
    }

    @Override
    public Set<String> getSubsystems() {
        return subsystems;
    }
}
