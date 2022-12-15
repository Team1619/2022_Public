package org.team1619.behavior;

import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.stream.Stream;
import java.util.Map;

/**
 * Zeroes the swerve modules for the drivetrain subsystem
 */

public class Drivetrain_Swerve_Zero extends BaseSwerve {

	private static final Logger logger = LogManager.getLogger(Drivetrain_Swerve_Zero.class);

	private final Timer timeoutTimer;

	private double zeroingThreshold;

	private int timeoutTime;

	public Drivetrain_Swerve_Zero(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, outputValues, robotConfiguration, true);

		timeoutTimer = new Timer();

		zeroingThreshold = 0.1;

		timeoutTime = 500;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		zeroingThreshold = config.getDouble("zeroing_threshold");

		timeoutTime = config.getInt("timeout_time");

		timeoutTimer.reset();
		timeoutTimer.start(timeoutTime);

		stopModules();

		sharedInputValues.setInputFlag(imu, "zero");
		sharedInputValues.setInputFlag(odometry, "zero");

		Stream.concat(angleOutputNames.stream(), speedOutputNames.stream()).forEach(output -> sharedOutputValues.setOutputFlag(output, "zero"));
	}

	@Override
	public void update() {
		if (!sharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")) {

			double maxWheelPosition = positionInputNames.stream().mapToDouble(sharedInputValues::getNumeric).map(Math::abs).max().getAsDouble();

			if (maxWheelPosition < zeroingThreshold && Math.abs(sharedInputValues.getVector(imu).get("trig_angle")) < zeroingThreshold) {

				sharedInputValues.setInputFlag(odometry, "zero");

				Map<String, Double> odometryValues = sharedInputValues.getVector(odometry);

				if (Math.abs(odometryValues.get("x")) < zeroingThreshold &&
						Math.abs(odometryValues.get("y")) < zeroingThreshold) {
					logger.debug("Drivetrain Zero -> Zeroed");
					sharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", true);
				}
			}

			if (timeoutTimer.isDone()) {
				logger.error("Drivetrain Zero -> Timed Out");

				timeoutTimer.reset();
				sharedInputValues.setInputFlag(odometry, "zero");
				sharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", true);
			}
		}
	}

	@Override
	public void dispose() {
	}

	@Override
	public boolean isDone() {
		return sharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
	}
}
