package org.team1619.state;

import edu.wpi.first.wpilibj.DriverStation;
import org.uacr.robot.AbstractRobotStatus;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.LimitedSizeQueue;
import org.uacr.utilities.RobotSystem;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;
import static java.lang.Math.*;
import java.util.Map;
import java.util.Queue;

/**
 * Sets flags and does global math and logic for competition bot
 */

public class RobotStatus extends AbstractRobotStatus {

	private static final Logger logger = LogManager.getLogger(RobotStatus.class);

	private static final double TARGET_HEIGHT = 102.75; // inches - center of reflective tape is 1.25" below the full height of 104"
	private static final double LIMELIGHT_HEIGHT = 33.75; // inches
	private static final double LIMELIGHT_ANGLE = 26.5; // degrees off horizontal - +1.0 degree = -5.0 inches
	private static final double TARGET_RADIUS = 24; // inches from center (0,0) to target tape
	private static final double LIMELIGHT_DISTANCE_FROM_ROBOT_CENTER = 2;
	private static final double TARGET_MIN_DISTANCE = 60; // closest distance in inches
	private static final double TARGET_MAX_DISTANCE = 320; // farthest distance in inches
	private final String fusedOdometry;
	private Queue<Double> limelightTyValues;

	private static final double MAX_TEMPERATURE = 85;

	private String limelightPnp;
	private String pnpButton;
	private String primeButton;
	private String shootButton;
	private String primeAgainstHubButton;
	private String primeLowButton;
	private String primeLaunchPad;
	private String ledButton;
	private String fusedOdometryLockOverideButton;
	private boolean ledOn;
	private String limelightAlignmentToggleButton;
	private Timer blinkTimer;
	private int validDistanceCalculationLimelightMinTxAngle;
	private double limelightTargetMaxAngle;

	public RobotStatus(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
		super(inputValues, outputValues, robotConfiguration);

		primeButton = robotConfiguration.getString("global_shoot", "prime_button");
		primeAgainstHubButton = robotConfiguration.getString("global_shoot", "prime_against_hub_button");
		primeLowButton = robotConfiguration.getString("global_shoot", "prime_low_button");
		primeLaunchPad = robotConfiguration.getString("global_shoot", "prime_launchpad_button");
		shootButton = robotConfiguration.getString("global_shoot", "shoot_button");
		limelightAlignmentToggleButton = robotConfiguration.getString("global_shoot", "limelight_alignment_toggle_button");
		ledButton = robotConfiguration.getString("global_all", "led_on_button");
		limelightTyValues = new LimitedSizeQueue<>(30);
		fusedOdometry = robotConfiguration.getString("global_drivetrain_swerve", "fused_odometry");
		sharedInputValues.setBoolean("ipb_robot_use_odometry_alignment", true);
		blinkTimer = new Timer();
		validDistanceCalculationLimelightMinTxAngle = robotConfiguration.getInt("global_targeting","valid_distance_calculation_limelight_min_tx_angle");
		pnpButton = robotConfiguration.getString("global_all", "pnp_button");
		limelightPnp = "pnp-main";
		fusedOdometryLockOverideButton = robotConfiguration.getString("global_shoot", "fused_odometry_lock_override_button");
		limelightTargetMaxAngle = robotConfiguration.getDouble("global_shoot", "limelight_target_max_angle");

	}

	@Override
	public void initialize() {
		// Zero
		if (!sharedInputValues.getBoolean("ipb_robot_has_been_zeroed")) {
			sharedInputValues.setBoolean("ipb_drivetrain_has_been_zeroed", false);
			sharedInputValues.setBoolean("ipb_climber_has_been_zeroed", false);
			sharedInputValues.setBoolean("ipb_collector_has_been_zeroed", false);
			sharedInputValues.setBoolean("ipb_flywheel_has_been_zeroed", false);
			sharedInputValues.setBoolean("ipb_hood_has_been_zeroed", false);
			sharedInputValues.setBoolean("ipb_elevator_has_been_zeroed", false);
		}
		if (RobotSystem.getRuntimeMode() == RobotSystem.RuntimeMode.SIM_MODE){
			sharedInputValues.setString("ips_alliance_color", "Blue");
		}
		else {
			sharedInputValues.setString("ips_alliance_color", DriverStation.getAlliance().toString());
		}

		sharedInputValues.setBoolean("ipb_drivetrain_safe_temperature", (sharedInputValues.getNumeric("ipn_drivetrain_front_right_speed_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_front_left_speed_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_back_left_speed_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_back_right_speed_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_front_right_angle_motor_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_front_left_angle_motor_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_back_left_angle_motor_temperature") < MAX_TEMPERATURE) &&
				(sharedInputValues.getNumeric("ipn_drivetrain_back_right_angle_motor_temperature") < MAX_TEMPERATURE));

		// Limelight
		ledOn = false;
		limelightPnp = "pnp-main";
		sharedInputValues.setInputFlag("ipv_limelight_shooter", limelightPnp);
		sharedInputValues.setBoolean("ipb_robot_use_odometry_alignment", true);
	}

	@Override
	public void disabledUpdate() {

		// Toggle Limelight light On/Off when disabled
		if (sharedInputValues.getBooleanRisingEdge(ledButton)) {
			sharedInputValues.setBoolean("ipb_limelight_on", !sharedInputValues.getBoolean("ipb_limelight_on"));
		}
		if (sharedInputValues.getBoolean("ipb_limelight_on")) {
			sharedInputValues.setInputFlag("ipv_limelight_shooter", "led-on");
		} else {
			sharedInputValues.setInputFlag("ipv_limelight_shooter", "led-off");
		}

		calculateDistanceToCenter();

		sharedInputValues.setString("ips_intake_ball_color", getIntakeColor());

		if (sharedInputValues.getBooleanRisingEdge(pnpButton)) {
			if (limelightPnp.equals("pnp-secondary")) {
				limelightPnp = "pnp-main";
				sharedInputValues.setInputFlag("ipv_limelight_shooter", "pnp-main");
			} else if (limelightPnp.equals("pnp-main")) {
				limelightPnp = "pnp-secondary";
				sharedInputValues.setInputFlag("ipv_limelight_shooter", "pnp-secondary");
			}
		}
	}

	@Override
	public void update() {

		if (!sharedInputValues.getBoolean("ipb_robot_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_drivetrain_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_climber_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_collector_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_flywheel_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_hood_has_been_zeroed")
				&& sharedInputValues.getBoolean("ipb_elevator_has_been_zeroed")) {
			sharedInputValues.setBoolean("ipb_robot_has_been_zeroed", true);
		}

		if (sharedInputValues.getBooleanRisingEdge(limelightAlignmentToggleButton)) {
			sharedInputValues.setBoolean("ipb_robot_use_odometry_alignment", !sharedInputValues.getBoolean("ipb_robot_use_odometry_alignment"));
		}

		// PRIME TO SHOOT
		Boolean primedToShoot;
		double absoluteUpdateValid = sharedInputValues.getVector(fusedOdometry).get("valid");
		if(sharedInputValues.getBoolean(primeLaunchPad) || sharedInputValues.getBoolean(primeAgainstHubButton) || sharedInputValues.getBoolean(primeLowButton)){
			primedToShoot = (sharedInputValues.getBoolean("ipb_flywheel_primed") && sharedInputValues.getBoolean("ipb_hood_primed"));
		} else {
			primedToShoot = ((absoluteUpdateValid > 0.0) || sharedInputValues.getBoolean(fusedOdometryLockOverideButton)) && sharedInputValues.getBoolean("ipb_drivetrain_primed") && sharedInputValues.getBoolean("ipb_flywheel_primed") && sharedInputValues.getBoolean("ipb_hood_primed");
		}
		sharedInputValues.setBoolean("ipb_primed_to_shoot", primedToShoot);

		if (sharedInputValues.getBoolean("ipb_primed_to_shoot")){
			sharedOutputValues.setNumeric("opn_relay_leds", "",1);
		}
		else if (sharedInputValues.getBoolean("ipb_loaded")) {
			sharedOutputValues.setNumeric("opn_relay_leds", "", -1);
		}
		else{
			sharedOutputValues.setNumeric("opn_relay_leds", "",0);
		}

		// LIMELIGHT LEDs TOGGLE
		if (sharedInputValues.getBooleanRisingEdge(ledButton)) {
			sharedInputValues.setBoolean("ipb_limelight_on", !sharedInputValues.getBoolean("ipb_limelight_on"));
		}
		if (sharedInputValues.getBoolean("ipb_limelight_on") || sharedInputValues.getBoolean("ipb_limelight_prime_on") ) {
			sharedInputValues.setInputFlag("ipv_limelight_shooter", "led-on");
		} else {
			sharedInputValues.setInputFlag("ipv_limelight_shooter", "led-off");
		}

		calculateDistanceToCenter();

		// Truncating values for display on the dashboard match screen
		sharedInputValues.setNumeric("match_flywheel_velocity", (int)sharedInputValues.getNumeric("ipn_flywheel_primary_velocity"));
		double hoodPosition = sharedInputValues.getNumeric("ipn_hood_position");
		hoodPosition = 0.5*(round(hoodPosition/0.5));
		sharedInputValues.setNumeric("match_hood_position", hoodPosition);

		sharedInputValues.setString("ips_intake_ball_color", getIntakeColor());

		if (sharedInputValues.getBooleanRisingEdge(pnpButton)) {
			if (limelightPnp.equals("pnp-secondary")) {
				limelightPnp = "pnp-main";
				sharedInputValues.setInputFlag("ipv_limelight_shooter", "pnp-main");
			} else if (limelightPnp.equals("pnp-main")) {
				limelightPnp = "pnp-secondary";
				sharedInputValues.setInputFlag("ipv_limelight_shooter", "pnp-secondary");
			}
		}
	}

	@Override
	public void dispose() {
		sharedInputValues.setInputFlag("ipv_limelight_shooter", "led-off");
	}

	private void calculateDistanceToCenter(){
		calculateDistanceToCenterWithLimelight();
		calculateDistanceToCenterWithOdometry();
		if(sharedInputValues.getBoolean("ipb_robot_use_odometry_alignment")){
			sharedInputValues.setNumeric("ipn_robot_distance_to_center_inches", sharedInputValues.getNumeric("ipn_odometry_distance_to_center_inches"));
		} else {
			sharedInputValues.setNumeric("ipn_robot_distance_to_center_inches", sharedInputValues.getNumeric("ipn_limelight_distance_to_center_inches"));
		}
	}

	private void calculateDistanceToCenterWithOdometry() {
		double x = sharedInputValues.getVector(fusedOdometry).get("x");
		double y = sharedInputValues.getVector(fusedOdometry).get("y");
		double heading = sharedInputValues.getVector(fusedOdometry).get("heading_acc");

		double distanceToCenterOdometry = sqrt((x*x)+(y*y));

		double desiredAngle = toDegrees(atan2(y, x));
		desiredAngle = ((desiredAngle + 180) % 360);
		double angleToCenterOdometry = ((((desiredAngle - heading) + 180) % 360) - 180 );

		// protect from unreasonable numbers
		if ((distanceToCenterOdometry >= 0 && distanceToCenterOdometry <= 360)) {
			sharedInputValues.setNumeric("ipn_odometry_distance_to_center_inches", distanceToCenterOdometry);
		} else {
			sharedInputValues.setNumeric("ipn_odometry_distance_to_center_inches", -1.0);
		}

		sharedInputValues.setNumeric("ipn_odometry_angle_to_center", angleToCenterOdometry);

		boolean limelightTargetInView = abs(angleToCenterOdometry) <= limelightTargetMaxAngle;
		sharedInputValues.setBoolean("ipb_limelight_target_in_view", limelightTargetInView);
	}

	private void calculateDistanceToCenterWithLimelight(){

		Map<String, Double> shooterLimelightValues = sharedInputValues.getVector("ipv_limelight_shooter");

		boolean isLocked = false;
		double limelightDistanceInches =-1.0;

		if(shooterLimelightValues.getOrDefault("tv", 0.0) > 0) {
			double ty = shooterLimelightValues.getOrDefault("ty", 0.0);
			double tx = shooterLimelightValues.getOrDefault("tx", 0.0);
			sharedInputValues.setNumeric("ipn_limelight_ty", ty);
			sharedInputValues.setNumeric("ipn_limelight_tx", tx);

			// Calculate distance (top of target height - limelight height) / tan(limelight angle + ty)
			if (abs(tx) < validDistanceCalculationLimelightMinTxAngle) {
				double distance = ((TARGET_HEIGHT - LIMELIGHT_HEIGHT) / tan(toRadians(LIMELIGHT_ANGLE + ty)));
				distance += TARGET_RADIUS + LIMELIGHT_DISTANCE_FROM_ROBOT_CENTER;

				if (distance >= TARGET_MIN_DISTANCE && distance <= TARGET_MAX_DISTANCE) {
					limelightDistanceInches = distance;
					isLocked = true;
				}
			}
		}

		sharedInputValues.setBoolean("ipb_limelight_locked", isLocked);
		sharedInputValues.setNumeric("ipn_limelight_distance_to_center_inches", limelightDistanceInches);
	}

	private String getIntakeColor() {
		Map<String, Double> colorReading = sharedInputValues.getVector("ipv_color_sensor");
		// Color readings are normalized, anything greater than 0.40 is the most likely color
		if (colorReading.get("red") > 0.38) {
			return DriverStation.Alliance.Red.toString();
		} else if (colorReading.get("blue") > 0.38) {
			return DriverStation.Alliance.Blue.toString();
		}
		return DriverStation.Alliance.Invalid.toString();
	}
}
