package org.team1619.models.outputs.numeric.robot;

import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.BaseTalon;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.sensors.SensorVelocityMeasPeriod;
import org.team1619.models.outputs.numeric.Talon;
import org.uacr.shared.abstractions.HardwareFactory;
import org.uacr.shared.abstractions.InputValues;
import org.uacr.utilities.Config;

/**
 * RobotTalon extends Talon, and controls talon motor controllers on the robot
 */

public class RobotTalon extends Talon {

    protected static final int CAN_TIMEOUT_MILLISECONDS = 10;

    protected final HardwareFactory hardwareFactory;
    protected final BaseTalon motor;

    public RobotTalon(Object name, Config config, HardwareFactory hardwareFactory, InputValues inputValues) {
        super(name, config, inputValues);

        this.hardwareFactory = hardwareFactory;

        Class<? extends BaseTalon> motorType = config.getString("type", "srx").equalsIgnoreCase("fx") ? TalonFX.class : TalonSRX.class;

        motor = hardwareFactory.get(motorType, deviceNumber);

        motor.configFactoryDefault(CAN_TIMEOUT_MILLISECONDS);

        motor.setInverted(isInverted);
        motor.setNeutralMode(isBrakeModeEnabled ? NeutralMode.Brake : NeutralMode.Coast);

        motor.setSensorPhase(sensorInverted);

        SupplyCurrentLimitConfiguration currentLimitConfiguration = new SupplyCurrentLimitConfiguration();
        currentLimitConfiguration.enable = currentLimitEnabled;
        currentLimitConfiguration.currentLimit = continuousCurrentLimitAmps;
        currentLimitConfiguration.triggerThresholdCurrent = peakCurrentLimitAmps;
        currentLimitConfiguration.triggerThresholdTime = peakCurrentDurationSeconds;
        motor.configSupplyCurrentLimit(currentLimitConfiguration, CAN_TIMEOUT_MILLISECONDS);

        // Sets the time between messages for specific message types on the CAN bus
        // https://docs.ctre-phoenix.com/en/latest/ch18_CommonAPI.html#motor-controllers
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, Status1CANbusUpdatePeriod);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, Status2CANbusUpdatePeriod);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_Brushless_Current, BrushlessCurrentCANbusUpdatePeriod);

        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_3_Quadrature, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_4_AinTempVbat, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_8_PulseWidth, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_10_MotionMagic, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_12_Feedback1, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_13_Base_PIDF0, 255);
        motor.setStatusFramePeriod(StatusFrameEnhanced.Status_14_Turn_PIDF1, 255);

        motor.configVelocityMeasurementPeriod(SensorVelocityMeasPeriod.Period_25Ms, CAN_TIMEOUT_MILLISECONDS);

        if(hasEncoder) {
            switch (feedbackDevice) {
                case "quad_encoder":
                    motor.configSelectedFeedbackSensor(FeedbackDevice.QuadEncoder, 0, CAN_TIMEOUT_MILLISECONDS);
                    break;
                case "internal_encoder":
                    motor.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, 0, CAN_TIMEOUT_MILLISECONDS);
                    break;
                case "cancoder":
                    motor.configRemoteFeedbackFilter(config.getInt("encoder_number"), RemoteSensorSource.CANCoder, 0, CAN_TIMEOUT_MILLISECONDS);
                    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, CAN_TIMEOUT_MILLISECONDS);
                    break;
                case "remote_talon":
                    motor.configRemoteFeedbackFilter(config.getInt("encoder_number"), RemoteSensorSource.TalonSRX_SelectedSensor, 0, CAN_TIMEOUT_MILLISECONDS);
                    motor.configSelectedFeedbackSensor(FeedbackDevice.RemoteSensor0, 0, CAN_TIMEOUT_MILLISECONDS);
                    break;
                default:
                    throw new RuntimeException("Invalid configuration for Talon feedback device: " + feedbackDevice);
            }
        }

        motor.configForwardLimitSwitchSource(forwardLimitSwitchEnabled ? LimitSwitchSource.FeedbackConnector : LimitSwitchSource.Deactivated, forwardLimitSwitchNormallyClosed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen, CAN_TIMEOUT_MILLISECONDS);
        motor.configReverseLimitSwitchSource(reverseLimitSwitchEnabled ? LimitSwitchSource.FeedbackConnector : LimitSwitchSource.Deactivated, reverseLimitSwitchNormallyClosed ? LimitSwitchNormal.NormallyClosed : LimitSwitchNormal.NormallyOpen, CAN_TIMEOUT_MILLISECONDS);
    }

    @Override
    public void processFlag(String flag) {
        if (flag.equals("zero")) {
            zeroSensor();
        }
        else if (flag.equals("coast")) {
            motor.setNeutralMode(NeutralMode.Coast);
        }
        else if (flag.equals("brake")) {
            motor.setNeutralMode(NeutralMode.Brake);
        }
    }

    @Override
    public void setHardware(String outputType, double outputValue, String profile) {
         if (readPosition) {
            readEncoderPosition();
        }
        if (readVelocity) {
            readEncoderVelocity();
        }
        if (readTemperature) {
            readMotorTemperature();
        }

        switch (outputType) {
            case "percent":
                motor.set(ControlMode.PercentOutput, outputValue * percentScalar);
                break;
            case "follower":
                motor.follow(hardwareFactory.get(TalonSRX.class, (int) outputValue), FollowerType.PercentOutput);
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_1_General, 255);
                motor.setStatusFramePeriod(StatusFrameEnhanced.Status_2_Feedback0, 255);
                break;
            case "velocity":
                setProfile(profile, velocityScalar * 10);

                motor.set(ControlMode.Velocity, (outputValue / velocityScalar) / 10);
                break;
            case "position":
                setProfile(profile, positionScalar);

                motor.set(ControlMode.Position, outputValue / positionScalar);
                break;
            case "motion_magic":
                setProfile(profile, positionScalar);

                motor.set(ControlMode.MotionMagic, outputValue / positionScalar);
                break;
            default:
                throw new RuntimeException("No output type " + outputType + " for TalonSRX");
        }
    }

    @Override
    public double getSensorPosition() {
        return motor.getSelectedSensorPosition(0) * positionScalar;
    }

    @Override
    public double getSensorVelocity() {
        return motor.getSelectedSensorVelocity(0) * 10 * velocityScalar;
    }

    @Override
    public double getMotorTemperature() {
        return motor.getTemperature();
    }

    @Override
    public void zeroSensor() {
        motor.setSelectedSensorPosition(0, 0, CAN_TIMEOUT_MILLISECONDS);
    }

    private void setProfile(String profileName, double scalar) {
        if (profileName.equals("none")) {
            throw new RuntimeException("PIDF Profile name must be specified");
        }

        if (profileName.equals(currentProfileName)) {
            return;
        }

        if (!profiles.containsKey(profileName)) {
            throw new RuntimeException("PIDF Profile " + profileName + " doesn't exist");
        }

        Config profile = new Config("pidf_config", profiles.get(profileName));

        motor.config_kP(0, profile.getDouble("p", 0.0), CAN_TIMEOUT_MILLISECONDS);
        motor.config_kI(0, profile.getDouble("i", 0.0), CAN_TIMEOUT_MILLISECONDS);
        motor.config_kD(0, profile.getDouble("d", 0.0), CAN_TIMEOUT_MILLISECONDS);
        motor.config_kF(0, profile.getDouble("f", 0.0), CAN_TIMEOUT_MILLISECONDS);
        motor.configMotionAcceleration(profile.getInt("acceleration", 0), CAN_TIMEOUT_MILLISECONDS);
        motor.configMotionCruiseVelocity(profile.getInt("cruise_velocity", 0), CAN_TIMEOUT_MILLISECONDS);
        //S Curve values tend to cause slamming and jerking
        motor.configMotionSCurveStrength(profile.getInt("s_curve", 0), CAN_TIMEOUT_MILLISECONDS);


        // IZone keeps the integrator empty until the error is within 'izone'. Thus i is not applied until close to the target
        // The izone value needs to be scaled base on the velocity or (position scaler divided by 10 as this is based on position change per 100ms)
        motor.config_IntegralZone(0, (profile.getDouble("izone", 0.0) / scalar), CAN_TIMEOUT_MILLISECONDS);

        currentProfileName = profileName;
    }
}
