package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import org.uacr.utilities.Timer;

import java.util.Set;

/**
 * Zeroes the hood subsystem
 */

public class Hood_Zero implements Behavior {

	private static final Logger logger = LogManager.getLogger(Hood_Zero.class);
	private static final Set<String> subsystems = Set.of("ss_hood");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final Timer maxTimeTimer;
	private final Timer waitForMotorToStartMovingTimer;
	private final Timer velocityNeedsToBeZeroForThisLongTimer;

	private double zeroingSpeed;
	private double zeroVelocityThreshold;
	private double currentHoodVelocity;

	private int maxTime;
	private int waitForMotorToStartMovingTime;
	private int velocityNeedsToZeroForThisLong;

	public Hood_Zero(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		maxTimeTimer = new Timer();
		waitForMotorToStartMovingTimer = new Timer();
		velocityNeedsToBeZeroForThisLongTimer = new Timer();

		zeroingSpeed = 0.0;
		zeroVelocityThreshold = 0.0;
		currentHoodVelocity = 0.0;

		maxTime = 0;
		waitForMotorToStartMovingTime = 0;
		velocityNeedsToZeroForThisLong = 0;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		maxTimeTimer.reset();
		waitForMotorToStartMovingTimer.reset();
		velocityNeedsToBeZeroForThisLongTimer.reset();

		zeroingSpeed = config.getDouble("zeroing_speed");
		zeroVelocityThreshold = config.getDouble("zeroing_threshold");
		currentHoodVelocity = 0.0;

		maxTime = config.getInt("zeroing_timeout_time");
		waitForMotorToStartMovingTime = config.getInt("zeroing_motor_start_time");
		velocityNeedsToZeroForThisLong = config.getInt("zeroing_velocity_wait_time");

		maxTimeTimer.start(maxTime);
		waitForMotorToStartMovingTimer.start(waitForMotorToStartMovingTime);
		sharedOutputValues.setNumeric("opn_hood", "percent", zeroingSpeed);
	}

	@Override
	public void update() {
		if (!sharedInputValues.getBoolean("ipb_hood_has_been_zeroed")) {
			if (maxTimeTimer.isDone()) {
				//checks if the timeout timer has been completed, if so then it will set its current position to zero.
				sharedOutputValues.setOutputFlag("opn_hood", "zero");
				sharedInputValues.setBoolean("ipb_hood_has_been_zeroed", true);
				sharedOutputValues.setNumeric("opn_hood", "percent", 0.0);

				logger.error("Hood Zero -> Zeroing Timed Out");
			} else if (velocityNeedsToBeZeroForThisLongTimer.isDone()) {
				sharedOutputValues.setOutputFlag("opn_hood", "zero");
				sharedInputValues.setBoolean("ipb_hood_has_been_zeroed", true);
				sharedOutputValues.setNumeric("opn_hood", "percent", 0.0);

				logger.debug("Hood Zero -> Zeroed");
			} else if (waitForMotorToStartMovingTimer.isDone()) {
				//Checks if the velocity is ever around the same value for a period of time, if it changes a substantial amount the timer will reset.
				currentHoodVelocity = sharedInputValues.getNumeric("ipn_hood_velocity");

				if (Math.abs(currentHoodVelocity) < zeroVelocityThreshold && !velocityNeedsToBeZeroForThisLongTimer.isStarted()) {
					velocityNeedsToBeZeroForThisLongTimer.start(velocityNeedsToZeroForThisLong);
				} else if (Math.abs(currentHoodVelocity) > zeroVelocityThreshold) {
					velocityNeedsToBeZeroForThisLongTimer.reset();
				}
			}
		}
	}

	@Override
	public void dispose() {
		sharedOutputValues.setNumeric("opn_hood", "percent", 0.0);
	}

	@Override
	public boolean isDone() {
		return sharedInputValues.getBoolean("ipb_hood_has_been_zeroed");
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
