// Copyright 2021-2023 FRC 6328
// http://github.com/Mechanical-Advantage
//
// This program is free software; you can redistribute it and/or
// modify it under the terms of the GNU General Public License
// version 3 as published by the Free Software Foundation or
// available in the root directory of this project.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
// GNU General Public License for more details.

package frc.robot.systems.drive.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;
import java.util.Queue;

/**
 * Module IO implementation for SparkMax drive motor controller, SparkMax turn motor controller (NEO
 * or NEO 550), and analog absolute encoder connected to the RIO
 *
 * <p>NOTE: This implementation should be used as a starting point and adapted to different hardware
 * configurations (e.g. If using a CANcoder, copy from "ModuleIOTalonFX")
 *
 * <p>To calibrate the absolute encoder offsets, point the modules straight (such that forward
 * motion on the drive motor will propel the robot forward) and copy the reported values from the
 * absolute encoders using AdvantageScope. These values are logged under
 * "/Drive/ModuleX/TurnAbsolutePositionRad"
 */
public class ModuleIOSparkMax implements ModuleIO {
  // Gear ratios for SDS MK4i L2, adjust as necessary
  private static final double DRIVE_GEAR_RATIO = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  private static final double TURN_GEAR_RATIO = 150.0 / 7.0;

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;
  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController turnController;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

  private final boolean isTurnMotorInverted = true;
  private double angleSetpointDegrees = 0;
  private double driveSetpointMetersPerSec = 0;
  private final Rotation2d absoluteEncoderOffset;

  public ModuleIOSparkMax(int index) {
    switch (index) {
      case 0:
        driveSparkMax = new CANSparkMax(1, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(2, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(0);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // MUST BE CALIBRATED
        break;
      case 1:
        driveSparkMax = new CANSparkMax(3, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(4, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(1);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // MUST BE CALIBRATED
        break;
      case 2:
        driveSparkMax = new CANSparkMax(5, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(6, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(2);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // MUST BE CALIBRATED
        break;
      case 3:
        driveSparkMax = new CANSparkMax(7, MotorType.kBrushless);
        turnSparkMax = new CANSparkMax(8, MotorType.kBrushless);
        turnAbsoluteEncoder = new CANcoder(3);
        absoluteEncoderOffset = Rotation2d.fromDegrees(0); // MUST BE CALIBRATED
        break;
      default:
        System.out.println("oopsie daisy");
        throw new RuntimeException("Invalid module index");
    }

    driveSparkMax.restoreFactoryDefaults();
    turnSparkMax.restoreFactoryDefaults();
    turnAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    canCoderConfig.MagnetSensor.SensorDirection =
        (!isTurnMotorInverted)
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = absoluteEncoderOffset.getDegrees();
    turnAbsoluteEncoder.getConfigurator().apply(canCoderConfig);

    driveSparkMax.setCANTimeout(250);
    turnSparkMax.setCANTimeout(250);

    driveEncoder = driveSparkMax.getEncoder();
    turnRelativeEncoder = turnSparkMax.getEncoder();

    turnSparkMax.setInverted(isTurnMotorInverted);
    driveSparkMax.setSmartCurrentLimit(40);
    turnSparkMax.setSmartCurrentLimit(30);
    driveSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.enableVoltageCompensation(12.0);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);

    turnRelativeEncoder.setPosition(0.0);
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);

    driveSparkMax.setCANTimeout(0);
    turnSparkMax.setCANTimeout(0);

    driveSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    turnSparkMax.setPeriodicFramePeriod(
        PeriodicFrame.kStatus2, (int) (1000.0 / Module.ODOMETRY_FREQUENCY));
    drivePositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue =
        SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveController = driveSparkMax.getPIDController();
    turnController = turnSparkMax.getPIDController();

    driveController.setP(1.0);
    driveController.setI(0.0);
    driveController.setD(0.0);
    driveController.setFeedbackDevice(driveEncoder);

    turnController.setP(1.0);
    turnController.setI(0.0);
    turnController.setD(0.0);
    turnController.setPositionPIDWrappingEnabled(true);
    turnController.setPositionPIDWrappingMinInput(0);
    turnController.setPositionPIDWrappingMaxInput(360);
    turnController.setFeedbackDevice(turnRelativeEncoder);

    driveSparkMax.burnFlash();
    turnSparkMax.burnFlash();
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters =
        (2 * Math.PI * driveEncoder.getPosition() * DriveConstants.kWheelWidth)
            / (DRIVE_GEAR_RATIO);
    inputs.driveVelocityMetersPerSec =
        (2 * Math.PI * driveEncoder.getVelocity() * DriveConstants.kWheelWidth)
            / (60 * DRIVE_GEAR_RATIO);
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveDesiredMetersPerSec = driveSetpointMetersPerSec;
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition =
        Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition().asSupplier().get());
    inputs.turnPosition =
        Rotation2d.fromRotations(turnRelativeEncoder.getPosition() / TURN_GEAR_RATIO);
    inputs.turnVelocityRadPerSec =
        Units.rotationsPerMinuteToRadiansPerSecond(turnRelativeEncoder.getVelocity())
            / TURN_GEAR_RATIO;
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnDesiredPosition = Rotation2d.fromDegrees(angleSetpointDegrees);
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> Units.rotationsToRadians(value) / DRIVE_GEAR_RATIO)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> Rotation2d.fromRotations(value / TURN_GEAR_RATIO))
            .toArray(Rotation2d[]::new);
    drivePositionQueue.clear();
    turnPositionQueue.clear();
  }

  @Override
  public void setDriveVoltage(double volts) {
    driveSparkMax.setVoltage(volts);
  }

  @Override
  public void setTurnVoltage(double volts) {
    turnSparkMax.setVoltage(volts);
  }

  @Override
  public void setDriveBrakeMode(boolean enable) {
    driveSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setTurnBrakeMode(boolean enable) {
    turnSparkMax.setIdleMode(enable ? IdleMode.kBrake : IdleMode.kCoast);
  }

  @Override
  public void setVelocity(double metersPerSecond) {
    driveSetpointMetersPerSec = metersPerSecond;
    driveController.setReference(metersPerSecond, ControlType.kVelocity);
  }

  @Override
  public void setPosition(double degrees) {
    angleSetpointDegrees = degrees;
    driveController.setReference(degrees, ControlType.kPosition);
  }
}
