package org.team1619.state.modelogic;

import org.uacr.models.state.State;
import org.uacr.robot.AbstractModeLogic;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

/**
 * Handles the isReady and isDone logic for teleop mode on competition bot
 */

public class TeleopModeLogic extends AbstractModeLogic {

	private static final Logger logger = LogManager.getLogger(TeleopModeLogic.class);

	private String collectButton;
	private String primeButton;
	private String primeAgainstHubButton;
	private String primeLowButton;
	private String primeLaunchpadButton;
	private String shootButton;
	private String ejectButton;
	private String dejamButton;
	private String stopElevatorCollectJoystick;

	private boolean isPriming;
	private boolean isPrimingAgainstHub;
	private boolean isPrimingLaunchpad;
	private boolean isCollecting;
	private boolean isShooting;
	private boolean isLowPrime;
	private boolean isEjecting;
	private boolean isDejamming;


	public TeleopModeLogic(InputValues inputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, robotConfiguration);

		collectButton = robotConfiguration.getString("global_collector", "collect_button");
		primeButton = robotConfiguration.getString("global_shoot", "prime_button");
		primeAgainstHubButton = robotConfiguration.getString("global_shoot", "prime_against_hub_button");
		primeLowButton = robotConfiguration.getString("global_shoot", "prime_low_button");
		primeLaunchpadButton = robotConfiguration.getString("global_shoot", "prime_launchpad_button");
		shootButton = robotConfiguration.getString("global_shoot", "shoot_button");
		ejectButton = robotConfiguration.getString("global_collector", "eject_button");
		dejamButton = robotConfiguration.getString("global_collector", "dejam_button");
		stopElevatorCollectJoystick = robotConfiguration.getString("global_elevator", "stop_elevator_collect_joystick");
	}

	@Override
	public void initialize() {
		logger.info("***** TELEOP *****");

		isPriming = false;
	}

	@Override
	public void update() {

		isCollecting = sharedInputValues.getBoolean(collectButton);

		if (sharedInputValues.getBoolean(primeButton) && !isShooting){
			isPriming = true;
			isPrimingAgainstHub = false;
			isLowPrime = false;
			isPrimingLaunchpad = false;
		}
		else {
			isPriming = false;
		}

		if (sharedInputValues.getBoolean(primeAgainstHubButton)){
			isCollecting = false;
			isPriming = false;
			isPrimingAgainstHub = true;
			isLowPrime = false;
			isPrimingLaunchpad = false;
		}
		else {
			isPrimingAgainstHub = false;
		}

		if(sharedInputValues.getBoolean(primeLowButton)) {
			isLowPrime = true;
			isPriming = false;
			isCollecting = false;
			isPrimingAgainstHub = false;
			isPrimingLaunchpad = false;
		} else {
			isLowPrime = false;
		}

		if(sharedInputValues.getBoolean(primeLaunchpadButton)){
			isCollecting = false;
			isPrimingLaunchpad = true;
			isPriming = false;
			isPrimingAgainstHub = false;
			isLowPrime = false;
		} else {
			isPrimingLaunchpad = false;
		}

		if(sharedInputValues.getBoolean(shootButton) && (sharedInputValues.getBoolean("ipb_primed_to_shoot") || isShooting)){
			isShooting = true;
			isPriming = false;
			isPrimingAgainstHub = false;
			isLowPrime = false;
			isPrimingLaunchpad = false;
		}
		else{
			isShooting = false;
		}

		if(sharedInputValues.getBoolean(ejectButton)){
			isEjecting = true;
			isCollecting = false;
			isPriming = false;
			isShooting = false;
			isLowPrime = false;
			isPrimingLaunchpad = false;
		}
		else{
			isEjecting = false;
		}

		if (sharedInputValues.getBoolean(dejamButton)){
			isDejamming = true;
			isEjecting = false;
			isCollecting = false;
			isPriming = false;
			isShooting = false;
			isLowPrime = false;
			isPrimingLaunchpad = false;
		}
		else{
			isDejamming = false;
		}
	}

	@Override
	public void dispose() {

	}

	@Override
	public boolean isReady(String name) {
		switch (name) {
			// Drivetrain
			case "st_drivetrain_zero":
				return !sharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed");
			case "st_drivetrain_swerve_align_odometry":
				return isPriming && (sharedInputValues.getBoolean("ipb_robot_use_odometry_alignment"));
			case "st_drivetrain_swerve_align_limelight":
				return isPriming && (!sharedInputValues.getBoolean("ipb_robot_use_odometry_alignment") && (sharedInputValues.getBoolean("ipb_limelight_locked")));

			// Climber
			case "st_climber_zero":
				return !sharedInputValues.getBoolean("ipb_climber_has_been_zeroed");

			// Collector
			case "st_collector_zero":
				return !sharedInputValues.getBoolean("ipb_collector_has_been_zeroed");

			// Flywheel
			case "st_flywheel_zero":
				return !sharedInputValues.getBoolean("ipb_flywheel_has_been_zeroed");

			//Hood
			case "st_hood_zero":
				return !sharedInputValues.getBoolean("ipb_hood_has_been_zeroed");

			//Elevator
			case "st_elevator_zero":
				return !sharedInputValues.getBoolean("ipb_elevator_has_been_zeroed");
			case "st_elevator_stop":
				return (sharedInputValues.getNumeric(stopElevatorCollectJoystick) <= -0.7);
			// Sequences
			case "pl_collect":
				return isCollecting;
			case "pl_eject":
				return isEjecting;
			case "pl_prime":
				return isPriming;
			case "pl_prime_against_hub":
				return isPrimingAgainstHub;
			case "pl_shoot":
				return isShooting;
			case "pl_prime_low_goal":
				return isLowPrime;
			case "pl_prime_launchpad":
				return isPrimingLaunchpad;
			case "pl_dejam":
				return isDejamming;

			default:
				return false;

		}
	}

	@Override
	public boolean isDone(String name, State state) {
		switch (name) {
			case "st_drivetrain_swerve_align_odometry":
				return !sharedInputValues.getBoolean(primeButton);
			case "st_drivetrain_swerve_align_limelight":
				return !sharedInputValues.getBoolean(primeButton) || !sharedInputValues.getBoolean("ipb_limelight_locked");

			case "st_elevator_stop":
				return !(sharedInputValues.getNumeric(stopElevatorCollectJoystick) <= -0.7);
			case "pl_collect":
				return !isCollecting;
			case "pl_eject":
				return !isEjecting;
			case "pl_prime":
				return !isPriming;
			case "pl_prime_against_hub":
				return !isPrimingAgainstHub;
			case "pl_shoot":
				return !isShooting;
			case "pl_prime_low_goal":
				return !isLowPrime;
			case "pl_prime_launchpad":
				return !isPrimingLaunchpad;
			case "pl_dejam":
				return !isDejamming;
			default:
				return state.isDone();
		}
	}
}
