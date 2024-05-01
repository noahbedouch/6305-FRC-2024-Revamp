package frc.lib.swerve;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Subsystem;
import frc.robot.Constants;

public class SwerveModule extends Subsystem {
    private final TalonFX driveMotor, angleMotor;
    private final CANcoder encoder;
    CANcoderConfiguration encoderConfig;
    TalonFXConfiguration driveConfig, angleConfig;
    private final double encoderZero;

    private final StatusSignal<Double> driveMotorVelocity;
    private final StatusSignal<Double> driveMotorPosition;
    private final StatusSignal<Double> driveMotorClosedLoopError;
    private final StatusSignal<Double> driveMotorTemperature;
    private final StatusSignal<Double> angleMotorVelocity;
    private final StatusSignal<Double> angleMotorPosition;
    private final StatusSignal<Double> angleMotorClosedLoopError;
    private final StatusSignal<Double> angleMotorTemperature;

    private final double driveMotorCoefficient = Math.PI * Constants.driveWheelDiameter * Constants.driveReduction;
    private final double anglePositionCoefficient = 2.0 * Math.PI * Constants.angleReduction;

    public SwerveModuleState targetModuleState;

    public SwerveModule(int driveID, int angleID, int encoderID, double encoderOffset) {
        driveMotor = new TalonFX(driveID, Constants.canivoreName);
        driveMotorVelocity = driveMotor.getRotorVelocity();
        driveMotorPosition = driveMotor.getRotorPosition();
        driveMotorClosedLoopError = driveMotor.getClosedLoopError();
        driveMotorTemperature = driveMotor.getDeviceTemp();

        angleMotor = new TalonFX(angleID, Constants.canivoreName);
        encoder = new CANcoder(encoderID, Constants.canivoreName);
        angleMotorVelocity = encoder.getVelocity();
        angleMotorPosition = encoder.getAbsolutePosition();
        angleMotorClosedLoopError = angleMotor.getClosedLoopError();
        angleMotorTemperature = angleMotor.getDeviceTemp();

        encoderZero = encoderOffset;

        configureTalons();
    }

    public void configureTalons() {
        driveConfig = new TalonFXConfiguration();

        driveConfig.CurrentLimits.SupplyCurrentLimit = 120;
        driveConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        driveConfig.CurrentLimits.StatorCurrentLimit = 120;
        driveConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        driveConfig.Slot0.kP = Constants.kPDrive;
        driveConfig.Slot0.kI = Constants.kIDrive;
        driveConfig.Slot0.kD = Constants.kDDrive;
        driveConfig.Slot0.kA = Constants.kADrive;
        driveConfig.Slot0.kS = Constants.kSDrive;
        driveConfig.Slot0.kV = Constants.kVDrive;

        driveMotor.getConfigurator().apply(driveConfig);

        angleConfig = new TalonFXConfiguration();

        angleConfig.CurrentLimits.SupplyCurrentLimit = 120;
        angleConfig.CurrentLimits.SupplyCurrentLimitEnable = true;
        angleConfig.CurrentLimits.StatorCurrentLimit = 120;
        angleConfig.CurrentLimits.StatorCurrentLimitEnable = false;

        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        angleConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;

        angleConfig.Slot0.kP = Constants.kPAngle;
        angleConfig.Slot0.kI = Constants.kIAngle;
        angleConfig.Slot0.kD = Constants.kDAngle;
        angleConfig.Slot0.kA = Constants.kAAngle;
        angleConfig.Slot0.kS = Constants.kSAngle;
        angleConfig.Slot0.kV = Constants.kVAngle;

        angleMotor.getConfigurator().apply(angleConfig);

        encoderConfig = new CANcoderConfiguration();

        encoderConfig.MagnetSensor.MagnetOffset = encoderZero;

        encoder.getConfigurator().apply(encoderConfig);
    }

    public Rotation2d getAngle() {
        return Rotation2d.fromRotations(encoder.getAbsolutePosition().getValueAsDouble());
    }

    public void setSteerCoastMode() {
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        angleMotor.getConfigurator().apply(angleConfig);
    }

    public void setSteerBrakeMode() {
        angleConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        angleMotor.getConfigurator().apply(angleConfig);
    }

    public void setDriveCoastMode() {
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Coast;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    public void setDriveBrakeMode() {
        driveConfig.MotorOutput.NeutralMode = NeutralModeValue.Brake;
        driveMotor.getConfigurator().apply(driveConfig);
    }

    public void setMaxOutput() {
        driveMotor.setControl(new VoltageOut(12.0));
    }

    public void zero() {
        angleMotor.setControl(new MotionMagicVoltage(0).withSlot(0));
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        targetModuleState = desiredState;

        driveMotor.setControl(new VelocityVoltage(targetModuleState.speedMetersPerSecond / driveMotorCoefficient).withSlot(0));
        angleMotor.setControl(new MotionMagicVoltage(targetModuleState.angle.getRadians() / anglePositionCoefficient).withSlot(0));
    }

    @Override
    public void readPeriodicOutputs() {}

    @Override
    public void stop() {
        driveMotor.stopMotor();
        angleMotor.stopMotor();
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry(boolean disabled) {}
}
