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
 * Manual control for the elevator; sets elevator speed
 */

public class Elevator_Manual implements Behavior {

	private static final Logger logger = LogManager.getLogger(Elevator_Manual.class);
	private static final Set<String> subsystems = Set.of("ss_elevator");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final String frontBeltEnabledButton;
	private final String backBeltEnabledButton;
	private final String frontBeltIncreaseButton;
	private final String frontBeltDecreaseButton;
	private final String backBeltIncreaseButton;
	private final String backBeltDecreaseButton;

	private double frontBeltSpeed;
	private double backBeltSpeed;
	private double collectBeltSpeed;
	private double adjustmentSpeed;

	private boolean frontBeltEnabled;
	private boolean backBeltEnabled;

	public Elevator_Manual(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		frontBeltEnabledButton = robotConfiguration.getString("global_manual", "manual_elevator_front_belt_enabled_button");
		backBeltEnabledButton = robotConfiguration.getString("global_manual", "manual_elevator_back_belt_enabled_button");
		frontBeltIncreaseButton = robotConfiguration.getString("global_manual", "manual_elevator_front_belt_increase_button");
		frontBeltDecreaseButton = robotConfiguration.getString("global_manual", "manual_elevator_front_belt_decrease_button");
		backBeltIncreaseButton = robotConfiguration.getString("global_manual", "manual_elevator_back_belt_increase_button");
		backBeltDecreaseButton = robotConfiguration.getString("global_manual", "manual_elevator_back_belt_decrease_button");

		frontBeltSpeed = 0.0;
		backBeltSpeed = 0.0;
		collectBeltSpeed = 0.0;
		adjustmentSpeed = 0.0;

		frontBeltEnabled = false;
		backBeltEnabled = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		frontBeltSpeed = config.getDouble("front_belt_speed", 0.0);
		backBeltSpeed = config.getDouble("back_belt_speed", 0.0);
		collectBeltSpeed = config.getDouble("collector_belt_speed", 0.0);
		adjustmentSpeed = config.getDouble("adjustment_speed", 0.0);
	}

	@Override
	public void update() {
		if (sharedInputValues.getBooleanRisingEdge(frontBeltIncreaseButton)) {
			frontBeltSpeed += adjustmentSpeed;
		}
		if (sharedInputValues.getBooleanRisingEdge(frontBeltDecreaseButton)) {
			frontBeltSpeed -= adjustmentSpeed;
		}
		if (sharedInputValues.getBooleanRisingEdge(backBeltIncreaseButton)) {
			backBeltSpeed += adjustmentSpeed;
		}
 		if (sharedInputValues.getBooleanRisingEdge(backBeltDecreaseButton)) {
			backBeltSpeed -= adjustmentSpeed;
		}

		sharedInputValues.setNumeric("ipn_elevator_front_belt_speed", frontBeltSpeed);
		sharedInputValues.setNumeric("ipn_elevator_back_belt_speed", backBeltSpeed);

		if (sharedInputValues.getBooleanRisingEdge(frontBeltEnabledButton)) {
			frontBeltEnabled = !frontBeltEnabled;
		}
		if (sharedInputValues.getBooleanRisingEdge(backBeltEnabledButton)) {
			backBeltEnabled = !backBeltEnabled;
		}

		if (frontBeltEnabled) {
			sharedOutputValues.setNumeric("opn_elevator_front_belt", "percent", frontBeltSpeed);
		} else {
			sharedOutputValues.setNumeric("opn_elevator_front_belt", "percent", 0.0);
		}
		if (backBeltEnabled) {
			sharedOutputValues.setNumeric("opn_elevator_back_belt", "percent", backBeltSpeed);
		} else {
			sharedOutputValues.setNumeric("opn_elevator_back_belt", "percent", 0.0);
		}

		sharedOutputValues.setNumeric("opn_elevator_collector_belt", "percent", collectBeltSpeed);
	}

	@Override
	public void dispose() {
		sharedOutputValues.setNumeric("opn_elevator_front_belt", "percent", 0.0);
		sharedOutputValues.setNumeric("opn_elevator_back_belt", "percent", 0.0);
		sharedOutputValues.setNumeric("opn_elevator_collector_belt", "percent", 0.0);
	}

	@Override
	public boolean isDone() {
		return true;
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
