package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;
import java.util.ArrayList;
import java.util.Map;

/**
 * Controls the hood subsystem, using the state interrupt system
 */

public class Hood_States implements Behavior {

	private static final Logger logger = LogManager.getLogger(Hood_States.class);
	private static final Set<String> subsystems = Set.of("ss_hood");

	private final InputValues sharedInputValues;
	private final OutputValues sharedOutputValues;

	private final ArrayList<Double> angleDistances;
	private final ArrayList<Double> angleProfile;

	private final double MIN_HOOD_ANGLE = 0.0;
	private final double MAX_HOOD_ANGLE = 26.0;

	private final double errorThreshold;
	private final double adjustAmount;

	private final String incrementButton;
	private final String decrementButton;

	private final int primedToShootFrames;

	private String distanceSource;

	private double desiredHoodPosition;
	private double hoodOffset;
	private double combinedPosition;
	private double hoodPosition;

	private int primedToShootCount;

	private boolean allowAdjust;
	private boolean isPrimedToShoot;

	public Hood_States(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		sharedInputValues = inputValues;
		sharedOutputValues = outputValues;

		angleProfile = new ArrayList<>();
		angleDistances = new ArrayList<>();

		for (Object p : robotConfiguration.getMap("global_hood", "angle_distance_profile").entrySet()) {
			Map.Entry<Double, Double> point = (Map.Entry<Double, Double>) p;
			angleDistances.add(point.getKey());
			angleProfile.add(point.getValue());
		}

		errorThreshold = robotConfiguration.getDouble("global_hood", "error_threshold");
		adjustAmount = robotConfiguration.getDouble("global_hood", "adjust_amount");

		incrementButton = robotConfiguration.getString("global_hood", "increment_button");
		decrementButton = robotConfiguration.getString("global_hood", "decrement_button");

		primedToShootFrames = robotConfiguration.getInt("global_targeting", "primed_to_shoot_frames");

		distanceSource = "none";

		desiredHoodPosition = 0.0;
		hoodOffset = 0.0;
		combinedPosition = 0.0;
		hoodPosition = 0.0;

		primedToShootCount = 0;

		isPrimedToShoot = false;
	}

	@Override
	public void initialize(String stateName, Config config) {
		logger.debug("Entering state {}", stateName);

		sharedInputValues.setBoolean("ipb_hood_primed",false);

		desiredHoodPosition = config.getDouble("hood_position", -1.0);
		allowAdjust = config.getBoolean("allow_adjust", false);
		distanceSource = config.getString("distance_source", "none");

		// Hold the current hood position if one is not specified in the state
		// This happens when switching from prime to shoot as shoot does not specify a hood_position
		if (desiredHoodPosition < 0) {
			desiredHoodPosition = hoodPosition;
		}

		primedToShootCount = 0;

		isPrimedToShoot = false;
	}

	@Override
	public void update() {
		if (distanceSource.equals("limelight")) {
			// If distance_source is set to "limelight" used the distance calculated by the limelight to calculate the hood angle.
			double robotDistanceInches = sharedInputValues.getNumeric("ipn_robot_distance_to_center_inches");

			if (robotDistanceInches > 0) {
				if (angleDistances.size() > 2) {
					if (robotDistanceInches < angleDistances.get(0)) {
						hoodPosition = angleProfile.get(0);
					} else if (robotDistanceInches > angleDistances.get(angleDistances.size() - 1)) {
						hoodPosition = angleProfile.get(angleProfile.size() - 1);
					} else {
						for (int index = 1; index < angleDistances.size(); index++) {
							if (angleDistances.get(index) > robotDistanceInches) {
								double angle1 = angleProfile.get(index - 1);
								double angle2 = angleProfile.get(index);
								double distance1 = angleDistances.get(index - 1);
								double distance2 = angleDistances.get(index);
								double slope = (angle2 - angle1) / (distance2 - distance1);
								double distancePastDistance1 = robotDistanceInches - distance1;
								hoodPosition = angle1 + slope * distancePastDistance1;
								break;
							}
						}
					}
				}
			} else {
				// Can't see limelight target - use default value for close up against the hub shot
				hoodPosition = desiredHoodPosition;
			}
		} else {
			// Otherwise, use the hood angle set in the state
			hoodPosition = desiredHoodPosition;
		}

		if (allowAdjust) {
			if (sharedInputValues.getBooleanRisingEdge(incrementButton)) {
				hoodOffset += adjustAmount;
			}
			if (sharedInputValues.getBooleanRisingEdge(decrementButton)) {
				hoodOffset -= adjustAmount;
			}
		}

		sharedInputValues.setNumeric("ipn_hood_offset", hoodOffset);

		combinedPosition = hoodPosition + hoodOffset;

		// protect from going out of hood range
		if (combinedPosition > MAX_HOOD_ANGLE) {
			combinedPosition = MAX_HOOD_ANGLE;
		}
		if (combinedPosition < MIN_HOOD_ANGLE) {
			combinedPosition = MIN_HOOD_ANGLE;
		}

		boolean isPrimed = Math.abs(combinedPosition - sharedInputValues.getNumeric("ipn_hood_position")) <= errorThreshold;

		if (isPrimed) {
			primedToShootCount ++;
			if (primedToShootCount > primedToShootFrames) {
				isPrimedToShoot = true;
			}
		} else {
			primedToShootCount = 0;
			isPrimedToShoot = false;
		}

		sharedInputValues.setBoolean("ipb_hood_primed", isPrimedToShoot);

		sharedOutputValues.setNumeric("opn_hood", "motion_magic", combinedPosition, "pr_hood");
	}

	@Override
	public void dispose() {
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
