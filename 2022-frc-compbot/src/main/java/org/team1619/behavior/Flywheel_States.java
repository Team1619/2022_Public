package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.ArrayList;
import java.util.Map;
import java.util.Set;

/**
 * Controls the flywheel subsystem, using the state interrupt system
 */

public class Flywheel_States implements Behavior {

	private static final Logger logger = LogManager.getLogger(Flywheel_States.class);
	private static final Set<String> subsystems = Set.of("ss_flywheel");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final ArrayList<Double> velocityDistances;
	private final ArrayList<Integer> velProfile;

	private final String incrementButton;
	private final String decrementButton;

	private final double velocityUpperLimit;
	private final double velocityLowerLimit;
	private final double adjustAmount;
	private final double finalVelocityError;

	private final int primedToShootFrames;

	private String velocityProfile;
	private String distanceSource;

	private double velocityOffset;
	private double desiredVelocity;
	private double velocity;

	private int primedToShootCount;

	private boolean coast;
	private boolean allowAdjust;
	private boolean isPrimedToShoot;
	private boolean limelightLedsOn;
	private boolean limelightLedsAutoToggle;

	public Flywheel_States(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		velocityDistances = new ArrayList<>();
		velProfile = new ArrayList<>();

		for (Object p : robotConfiguration.getMap("global_flywheel", "velocity_distance_profile").entrySet()) {
			Map.Entry<Double, Integer> point = (Map.Entry<Double, Integer>) p;
			velocityDistances.add(point.getKey());
			velProfile.add(point.getValue());
		}

		incrementButton = robotConfiguration.getString("global_flywheel", "velocity_increment_button");
		decrementButton = robotConfiguration.getString("global_flywheel", "velocity_decrement_button");

		velocityUpperLimit = robotConfiguration.getDouble("global_flywheel", "velocity_upper_limit");
		velocityLowerLimit = robotConfiguration.getDouble("global_flywheel", "velocity_lower_limit");
		adjustAmount = robotConfiguration.getDouble("global_flywheel", "velocity_adjust_amount");
		finalVelocityError = robotConfiguration.getDouble("global_flywheel", "final_velocity_error");

		primedToShootFrames = robotConfiguration.getInt("global_targeting", "primed_to_shoot_frames");

		velocityProfile = "";

		velocityOffset = 0.0;
		desiredVelocity = 0.0;
		velocity = 0.0;

		primedToShootCount = 0;

		coast = true;
		allowAdjust = false;
		isPrimedToShoot = false;
		limelightLedsOn = false;
		limelightLedsAutoToggle = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		velocityProfile = config.getString("velocity_profile", "");
		distanceSource = config.getString("distance_source", "none");

		desiredVelocity = config.getDouble("velocity", -1.0);

		coast = config.getBoolean("coast", true);
		allowAdjust = config.getBoolean("allow_adjust", false);
		limelightLedsOn = config.getBoolean("limelight_leds_on", false);
		limelightLedsAutoToggle = config.getBoolean("limelight_leds_auto_toggle", false);

		// Hold the current flywheel velocity if one is not specified in the state
		// This happens when switching from prime to shoot as shoot does not specify a velocity
		if (desiredVelocity < 0.0) {
			desiredVelocity = velocity;
		} else {
			isPrimedToShoot = false;
			primedToShootCount = 0;
		}

		sharedInputValues.setBoolean("ipb_flywheel_primed",false);

		// Turn Limelight LEDs on for priming - handles in RobotStatus
		sharedInputValues.setBoolean("ipb_limelight_prime_on", limelightLedsOn);
	}

	@Override
	public void update() {
		if (distanceSource.equals("limelight")) {
			// If distance_source is set to "limelight," use the distance calculated by the limelight to calculate the flywheel velocity.
			double robotDistanceInches = sharedInputValues.getNumeric("ipn_robot_distance_to_center_inches");

			if (robotDistanceInches > 0) {
				if (velocityDistances.size() > 2) {
					if (robotDistanceInches < velocityDistances.get(0)) {
						velocity = velProfile.get(0);
					} else if (robotDistanceInches > velocityDistances.get(velocityDistances.size() - 1)) {
						velocity = velProfile.get(velProfile.size() - 1);
					} else {
						for (int index = 1; index < velocityDistances.size(); index++) {
							if (velocityDistances.get(index) > robotDistanceInches) {
								double velocity1 = velProfile.get(index - 1);
								double velocity2 = velProfile.get(index);
								double distance1 = velocityDistances.get(index - 1);
								double distance2 = velocityDistances.get(index);
								double slope = (velocity2 - velocity1) / (distance2 - distance1);
								double distancePastDistance1 = robotDistanceInches - distance1;
								velocity = velocity1 + slope * distancePastDistance1;
								break;
							}
						}
					}
				}
			} else {
				// Can't see the limelight target - use default values for hub shot
				velocity = desiredVelocity;
			}
		} else {
			// Otherwise, use the velocity set in the state
			velocity = desiredVelocity;
		}

		// set brake mode - ignored in velocity mode
		String brakeMode = (coast) ? "coast" : "brake";
		sharedOutputValues.setOutputFlag("opn_flywheel", brakeMode);

		if (allowAdjust) {
			if (sharedInputValues.getBooleanRisingEdge(incrementButton)) {
				velocityOffset += adjustAmount;
			}
			if (sharedInputValues.getBooleanRisingEdge(decrementButton)) {
				velocityOffset -= adjustAmount;
			}
		}

		sharedInputValues.setNumeric("ipn_flywheel_offset", velocityOffset);

		String outputType;
		double motorValue;

		if (velocity == 0.0) {
			// stopping/
			outputType = "percent";
			motorValue = 0.0;
			isPrimedToShoot = false;
		} else {
			// velocity mode
			outputType = "velocity";
			motorValue = velocity;
			motorValue += velocityOffset;
			motorValue = Math.min(motorValue, velocityUpperLimit);
			motorValue = Math.max(motorValue, velocityLowerLimit);

			// Require at least 10 prime to shoot frames in a row to filter out when the flywheel speed over / under shoots
			boolean isPrimed = (Math.abs(motorValue - sharedInputValues.getNumeric("ipn_flywheel_primary_velocity")) <= finalVelocityError);

			if (isPrimed) {
				primedToShootCount ++;
				if (primedToShootCount > primedToShootFrames) {
					isPrimedToShoot = true;
				}
			} else {
				primedToShootCount = 0;
				isPrimedToShoot = false;
			}
		}

		if (limelightLedsAutoToggle) {
			sharedInputValues.setBoolean("ipb_limelight_prime_on", sharedInputValues.getBoolean("ipb_limelight_target_in_view"));
		}

		// Set motors
		sharedInputValues.setBoolean("ipb_flywheel_primed", isPrimedToShoot);
		sharedOutputValues.setNumeric("opn_flywheel", outputType, motorValue, velocityProfile);
	}

	@Override
	public void dispose() {
		sharedOutputValues.setOutputFlag("opn_flywheel", "coast");
		sharedOutputValues.setNumeric("opn_flywheel", "percent", 0.0);
	}

	@Override
	public boolean isDone() {
		return isPrimedToShoot;
	}

	@Override
	public Set<String> getSubsystems() {
		return subsystems;
	}
}
