package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.Config;
import org.uacr.utilities.Timer;
import org.uacr.utilities.logging.LogManager;
import org.uacr.utilities.logging.Logger;

import java.util.Set;

import static org.team1619.behavior.Elevator_States.BallColor.*;
import static org.team1619.behavior.Elevator_States.Beam.ABSENT;
import static org.team1619.behavior.Elevator_States.Beam.PRESENT;

/**
 * Behavior for controlling the elevator by reading sensor values
 * directly without tracking ball state between frames.
 */

public class Elevator_States implements Behavior {

    private static final Logger logger = LogManager.getLogger(Elevator_States.class);
    private static final Set<String> subsystems = Set.of("ss_elevator");

    private final InputValues sharedInputValues;
    private final OutputValues sharedOutputValues;

    private final Timer ejectTimer;

    private final String velocityProfile;
    private final String dumpOverrideButton;

    private final double upSpeedUpperBallMissing;

    private String ourAllianceColor;

    private double UP;
    private double DOWN;
    private double COLLECT_IN;
    private double COLLECT_OUT;

    private double backBeltHoldAdjustAmount;

    private int beltSpeed;
    private int collectorBeltSpeed;
    private int ejectDuration;

    private boolean isGuarding;
    private boolean isStopped;
    private boolean isEjecting;
    private boolean isDejamming;
    private boolean isShooting;
    private boolean overrideBallColors;
    private boolean overrideDumpSensor;

    public Elevator_States(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration) {
        sharedInputValues = inputValues;
        sharedOutputValues = outputValues;

        ejectTimer = new Timer();

        velocityProfile = "pr_belts";
        dumpOverrideButton = robotConfiguration.getString("global_elevator","dump_override_button");

        upSpeedUpperBallMissing = robotConfiguration.getDouble("global_elevator", "speed_while_shooting_and_missing_upper_ball");

        overrideBallColors = false;
        overrideDumpSensor = false;
    }

    @Override
    public void initialize(String stateName, Config config) {
        logger.debug("Entering state {}", stateName);

        ejectTimer.reset();

        backBeltHoldAdjustAmount = config.getDouble("back_belt_hold_adjust_amount", 0.0);

        ourAllianceColor = sharedInputValues.getString("ips_alliance_color");

        beltSpeed = config.getInt("belt_speed", 0);
        collectorBeltSpeed = config.getInt("collector_belt_speed", 0);
        ejectDuration = config.getInt("eject_duration", 0);

        isGuarding = config.getBoolean("guarding", true);
        isStopped = config.getBoolean("stop", false);
        isEjecting = config.getBoolean("ejecting", false);
        isDejamming = config.getBoolean("dejamming", false);
        isShooting = config.getBoolean("shooting", false);

        // We currently don't have any UP speed adjustments
        UP = beltSpeed;
        COLLECT_IN = collectorBeltSpeed;
    }

    enum Beam {PRESENT, ABSENT}

    enum BallColor {CORRECT, INCORRECT, INVALID}

    @Override
    public void update() {
        // read all inputs into local vars
        Beam top = sharedInputValues.getBoolean("ipb_elevator_top_beam_sensor") ? PRESENT : ABSENT;
        Beam bottom = sharedInputValues.getBoolean("ipb_elevator_bottom_beam_sensor") ? PRESENT : ABSENT;
        Beam hood = sharedInputValues.getBoolean("ipb_elevator_hood_beam_sensor") ? PRESENT : ABSENT;
        Beam dump = sharedInputValues.getBoolean("ipb_elevator_dump_beam_sensor") ? PRESENT : ABSENT;

        if (sharedInputValues.getBooleanRisingEdge(dumpOverrideButton)) {
            overrideDumpSensor = !overrideDumpSensor;
            overrideBallColors = !overrideBallColors;
        }

        if (overrideDumpSensor) {
            dump = ABSENT;
        }

        final BallColor ballColor;
        String colorSensor = sharedInputValues.getString("ips_intake_ball_color");

        if (!overrideBallColors) {
            if (colorSensor.equalsIgnoreCase(ourAllianceColor)) {
                ballColor = CORRECT;
            } else if (colorSensor.equalsIgnoreCase("Invalid")) {
                ballColor = INCORRECT;
            } else {
                ballColor = INCORRECT;
            }
        } else {
            ballColor = CORRECT;
        }

        // vars for outputs - declaring final ensures they get set exactly once below
        final double frontBelt;
        final double backBelt;
        final double collectorBelt;

        // Conditional logic is based on the state tables in this spreadsheet:
        // https://docs.google.com/spreadsheets/d/1ZQvbAqVioEi7ZLWrtlAr-AHW08o7k9h9gbK6WJmE5lc/edit#gid=0

        if (isEjecting && !ejectTimer.isStarted()) {
            ejectTimer.start(ejectDuration);
        }

        // See "down speed table" in the spreadsheet
        if (isDejamming || isEjecting) {
            // don't carry over down-speed adjustments from juggling when dejamming
            DOWN = -beltSpeed;
            COLLECT_OUT = -collectorBeltSpeed;
        } else if (top == PRESENT && hood == ABSENT && bottom != PRESENT) {
            // move ball up while juggling
            DOWN += backBeltHoldAdjustAmount;
        } else if (top == PRESENT && hood == PRESENT && bottom != PRESENT) {
            // move ball down while juggling
            DOWN -= backBeltHoldAdjustAmount;
        } else {
            // reset down-speed adjustments from juggling when no ball is present up top
            DOWN = -beltSpeed;
            COLLECT_OUT = -collectorBeltSpeed;
        }

        final double STOP = 0.0;

        // See "up/down table" in the spreadsheet
        if (isDejamming) {
            // sets the belt speeds backwards when dejamming
            frontBelt = DOWN;
            backBelt = DOWN;
            collectorBelt = COLLECT_OUT;
        } else if (isEjecting) {
            frontBelt = ejectTimer.isDone() ? STOP : DOWN;
            backBelt = STOP;
            collectorBelt = ejectTimer.isDone() ? STOP: COLLECT_OUT;
        } else if (isStopped) {
            frontBelt = STOP;
            backBelt = STOP;
            collectorBelt = STOP;
        } else if (dump == PRESENT) {
            // dump the incorrect color ball
            frontBelt = UP;
            backBelt = DOWN;
            collectorBelt = COLLECT_IN;
        } else if (isShooting) {
            // lets go!!
            collectorBelt = COLLECT_IN;
            if (top == ABSENT) {
                frontBelt = upSpeedUpperBallMissing;
                backBelt = upSpeedUpperBallMissing;
            } else {
                frontBelt = UP;
                backBelt = UP;
            }
        } else if (top == ABSENT && bottom == PRESENT && ballColor == CORRECT) {
            // moves the correct color ball up
            frontBelt = UP;
            backBelt = UP;
            collectorBelt = COLLECT_IN;
        } else if (top == PRESENT && bottom == PRESENT && ballColor == CORRECT) {
            // we have two balls of the correct color, so stop
            frontBelt = STOP;
            backBelt = STOP;
            collectorBelt = STOP;
        } else if (top == ABSENT && bottom == PRESENT && ballColor == INCORRECT) {
            // ejects bottom ball
            frontBelt = UP;
            backBelt = DOWN;
            collectorBelt = COLLECT_IN;
        } else if (top == PRESENT && bottom == PRESENT && ballColor == INCORRECT) {
            // juggles top ball, ejects incorrect ball if it exists
            frontBelt = UP;
            backBelt = DOWN;
            collectorBelt = COLLECT_IN;
        } else if (top == ABSENT && bottom == PRESENT && ballColor == INVALID) {
            // one unknown ball, wait for color fix
            frontBelt = STOP;
            backBelt = STOP;
            collectorBelt = STOP;
        } else if (top == PRESENT && bottom == PRESENT && ballColor == INVALID) {
            // good ball on the top, unknown ball bottom, wait for a color fix
            frontBelt = STOP;
            backBelt = STOP;
            collectorBelt = STOP;
        } else if (top == ABSENT && bottom == ABSENT) {
            // nothing in the robot, so be ready to elevate new ball
            frontBelt = UP;
            backBelt = UP;
            collectorBelt = COLLECT_IN;
        } else if (top == PRESENT && bottom == ABSENT) {
            // juggles the top ball
            frontBelt = UP;
            backBelt = DOWN;
            collectorBelt = COLLECT_IN;
        } else {
            logger.error("Unimplemented state in elevator update() - presuming UP is the best choice. ¯\\_(ツ)_/¯");
            frontBelt = UP;
            backBelt = UP;
            collectorBelt = COLLECT_IN;
        }

        // Outputs
        sharedInputValues.setBoolean("ipb_loaded", top == PRESENT && bottom == PRESENT && ballColor == CORRECT);
        sharedOutputValues.setBoolean("opb_ball_guard", isGuarding);
        sharedOutputValues.setBoolean("ipb_override_color_sensor", overrideBallColors);
        sharedOutputValues.setNumeric("opn_elevator_front_belt", "velocity", frontBelt, velocityProfile);
        sharedOutputValues.setNumeric("opn_elevator_back_belt", "velocity", backBelt, velocityProfile);
        sharedOutputValues.setNumeric("opn_elevator_collector_belt", "velocity", collectorBelt, velocityProfile);

        // values for debugging
        sharedInputValues.setString("ips_elevator_bb_top", top.toString());
        sharedInputValues.setString("ips_elevator_bb_bottom", bottom.toString());
        sharedInputValues.setString("ips_elevator_bb_hood", hood.toString());
        sharedInputValues.setString("ips_elevator_bb_dump", dump.toString());
        sharedInputValues.setString("ips_elevator_ball_color", ballColor.toString());
        sharedInputValues.setString("ips_elevator_color_sensor", colorSensor);
        sharedInputValues.setString("ips_elevator_alliance_color", ourAllianceColor);
        sharedInputValues.setNumeric("ips_elevator_down_speed", DOWN);
        sharedInputValues.setNumeric("ips_elevator_collector_speed", collectorBeltSpeed);
    }

    @Override
    public void dispose() {
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
