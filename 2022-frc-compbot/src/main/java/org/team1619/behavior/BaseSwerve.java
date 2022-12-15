package org.team1619.behavior;

import org.uacr.models.behavior.Behavior;
import org.uacr.models.exceptions.ConfigurationException;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.shared.abstractions.OutputValues;
import org.uacr.shared.abstractions.RobotConfiguration;
import org.uacr.utilities.closedloopcontroller.ClosedLoopController;
import org.uacr.utilities.purepursuit.Vector;
import org.uacr.utilities.purepursuit.VectorList;

import java.util.Collections;
import java.util.List;
import java.util.Set;
import java.util.stream.Collectors;
import java.util.stream.Stream;

public abstract class BaseSwerve implements Behavior {

    protected static final Set<String> subsystems = Set.of("ss_drivetrain");

    protected final InputValues sharedInputValues;
    protected final OutputValues sharedOutputValues;

    protected final VectorList modulePositions;
    protected final VectorList moduleRotationDirections;
    private final VectorList currentModuleVectors;

    protected final List<String> angleInputNames;
    protected final List<String> positionInputNames;
    protected final List<String> angleOutputNames;
    protected final List<String> speedOutputNames;

    private final ClosedLoopController headingController;

    protected final String imu;
    protected final String limelight;
    protected final String odometry;
    protected final String fusedOdometry;

    protected final double maxModuleVelocity;

    protected final boolean useAngleDifferenceScalar;

    protected double headingOffset;
    protected double originalHeadingAngle;
    protected double currentMaxModuleVelocity;

    private String headingMode;

    public BaseSwerve(InputValues inputValues, OutputValues outputValues, RobotConfiguration robotConfiguration, boolean useAngleDifferenceScalar) {
        sharedInputValues = inputValues;
        sharedOutputValues = outputValues;

        this.useAngleDifferenceScalar = useAngleDifferenceScalar;

        maxModuleVelocity = robotConfiguration.getDouble("global_drivetrain_swerve", "max_module_velocity");
        currentMaxModuleVelocity = maxModuleVelocity;

        modulePositions = new VectorList(((List<List<Double>>) robotConfiguration.getList("global_drivetrain_swerve", "module_positions")).stream().map(Vector::new).collect(Collectors.toList()));

        currentModuleVectors = new VectorList(Collections.nCopies(4, new Vector()));

        // Create a set of vectors at right angles to the corners of the robot to use to calculate rotation vectors
        moduleRotationDirections = modulePositions.copy().normalizeAll().rotateAll(90);

        headingController = new ClosedLoopController(robotConfiguration.getString("global_drivetrain_swerve", "heading_controller"));

        angleInputNames = robotConfiguration.getList("global_drivetrain_swerve", "input_angle_names");
        positionInputNames = robotConfiguration.getList("global_drivetrain_swerve", "input_position_names");
        angleOutputNames = robotConfiguration.getList("global_drivetrain_swerve", "output_angle_names");
        speedOutputNames = robotConfiguration.getList("global_drivetrain_swerve", "output_speed_names");

        imu = robotConfiguration.getString("global_drivetrain_swerve", "imu");
        limelight = robotConfiguration.getString("global_drivetrain_swerve", "limelight");
        odometry = robotConfiguration.getString("global_drivetrain_swerve", "odometry");
        fusedOdometry = robotConfiguration.getString("global_drivetrain_swerve", "fused_odometry");

        headingOffset = robotConfiguration.getDouble("global_drivetrain_swerve", "heading_angle_offset");
        originalHeadingAngle = headingOffset;

        headingMode = "none";
    }

    protected void setModulePowers(Vector translation, VectorList moduleRotationDirections, double rotationSpeed, String pidProfile) {
        setMotorPowers(calculateModuleVectors(translation, moduleRotationDirections, rotationSpeed), pidProfile);
    }

    protected void setModulePowers(Vector translation, double rotationSpeed, String pidProfile) {
        setModulePowers(translation, moduleRotationDirections, rotationSpeed, pidProfile);
    }

    protected void setModulePowers(Vector translation, String headingMode, double headingOutput, String headingProfile, String pidProfile) {
        switch (headingMode) {
            case "imu":
                if ("none".equals(headingProfile)) {
                    headingProfile = "imu";
                }
                break;
            case "angle":
                if ("none".equals(headingProfile)) {
                    headingProfile = "angle";
                }
                break;
            case "fused_odometry":
                if ("none".equals(headingProfile)) {
                    headingProfile = "fused_odometry";
                }
                break;
            case "limelight":
                if ("none".equals(headingProfile)) {
                    headingProfile = "limelight";
                }
                break;
            default:
                throw new ConfigurationException("Heading mode " + headingMode + " doesn't exist.");
        }

        headingController.setProfile(headingProfile);
        headingController.set(headingOutput);

        if (!this.headingMode.equals(headingMode)) {
            headingController.reset();
            this.headingMode = headingMode;
        }

        switch (headingMode) {
            case "imu":
                // When the headingMode is IMU-based, use the odometry's heading since the odometry keeps an offset
                // from the IMU when an auto path defines a non-zero start heading.
                setModulePowers(translation, headingController.getWithPID(sharedInputValues.getVector(odometry).get("heading_acc")), pidProfile);
                break;
            case "angle":
                setModulePowers(translation, headingController.getWithPID(sharedInputValues.getVector(odometry).get("heading")), pidProfile);
                break;
            case "fused_odometry":
                double heading = sharedInputValues.getVector(fusedOdometry).get("heading");
                double x = sharedInputValues.getVector(fusedOdometry).get("x");
                double y = sharedInputValues.getVector(fusedOdometry).get("y");
                double desiredAngle = Math.toDegrees(Math.atan2(y, x));
                desiredAngle = ((desiredAngle + 180) % 360);
                double deltaAngle = -1 * ((((desiredAngle - heading) + 180) % 360) - 180);
                sharedInputValues.setNumeric("ipn_fused_odometry_delta_angle", deltaAngle);
                setModulePowers(translation, headingController.getWithPID(deltaAngle + headingOffset), pidProfile);
                break;
            case "limelight":
                setModulePowers(translation, headingController.getWithPID(sharedInputValues.getVector(limelight).get("tx") + headingOffset), pidProfile);
                break;
        }
    }

    protected void setModulePowers(Vector translation, String headingMode, double headingOutput, String pidProfile) {
        setModulePowers(translation, headingMode, headingOutput, "none", pidProfile);
    }

    protected void stopModules() {
        Stream.concat(angleOutputNames.stream(), speedOutputNames.stream()).forEach(output -> sharedOutputValues.setNumeric(output, "percent", 0.0));
    }

    protected VectorList calculateModuleVectors(Vector translation, VectorList moduleRotationDirections, double rotationSpeed) {
        for (int i = 0; i < 4; i++) {
            currentModuleVectors.set(i, calculateModuleVector(currentModuleVectors.get(i), angleInputNames.get(i), translation, moduleRotationDirections.get(i), rotationSpeed));
        }

        return currentModuleVectors.copy();
    }

    protected VectorList calculateModuleVectors(Vector translation, double rotationSpeed) {
        return calculateModuleVectors(translation, moduleRotationDirections, rotationSpeed);
    }

    protected Vector calculateModuleVector(Vector last, String currentAngleInput, Vector translation, Vector rotationDirection, Double rotationScalar) {
        double currentModuleAngle = sharedInputValues.getVector(currentAngleInput).get("absolute_position");
        Vector current = new Vector(last.magnitude(), currentModuleAngle);
        Vector target = new Vector(translation.add(rotationDirection.scale(rotationScalar)));

        // When the joysticks are idle, move the wheel angles to their rotation angle so the robot can spin instantly and move in any direction as quickly as possible.
        if (target.magnitude() == 0.0) {
            target = new Vector(0, last.angle());
        }

        // If the difference between the target angle and the actual angle is more than 90 degrees, rotate 180 degrees and reverse the motor direction.
        // Ramp up the wheel velocity as the actual angle get closer to the target angle. This prevents the robot from being pulled off course.
        // The cosine is raised to the power of 3 so that the ramp increases faster as the delta in the angle approaches zero.
        double directionScalar = Math.pow(Math.cos(Math.toRadians(target.angle() - current.angle())), 3);

        if (!useAngleDifferenceScalar) {
            directionScalar = Math.signum(directionScalar);
        }

        if (directionScalar < 0) {
            target = target.rotate(180);
        }

        return target.scale(directionScalar);
    }

    protected void setMotorPowers(VectorList moduleVectors, String pidProfile) {
        // moduleVectors.autoScaleAll(VectorList.AutoScaleMode.SCALE_LARGEST_DOWN, 1.0);

        for (int m = 0; m < moduleVectors.size(); m++) {
            setMotorPower(m, moduleVectors.get(m), pidProfile);
        }
    }

    protected void setMotorPower(int moduleNumber, Vector moduleVector, String pidProfile) {
        sharedOutputValues.setNumeric(angleOutputNames.get(moduleNumber), "absolute_position", moduleVector.angle(), "pr_drive");
        sharedOutputValues.setNumeric(speedOutputNames.get(moduleNumber), "velocity", moduleVector.magnitude() * currentMaxModuleVelocity, pidProfile);
    }

    @Override
    public Set<String> getSubsystems() {
        return subsystems;
    }
}
