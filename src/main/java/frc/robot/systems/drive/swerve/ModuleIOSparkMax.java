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
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
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

  private final CANSparkMax driveSparkMax;
  private final CANSparkMax turnSparkMax;
  private final SparkMaxPIDController drivePIDController;
  private final SparkMaxPIDController turnPIDController;
  private final SimpleMotorFeedforward driveFF;
  private final SimpleMotorFeedforward turnFF;

  private final RelativeEncoder driveEncoder;
  private final RelativeEncoder turnRelativeEncoder;
  private final CANcoder turnAbsoluteEncoder;
  private final Queue<Double> drivePositionQueue;
  private final Queue<Double> turnPositionQueue;

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

    turnAbsoluteEncoder.getConfigurator().apply(new CANcoderConfiguration());
    turnAbsoluteEncoder.getConfigurator().apply(DriveConstants.getCANCoderConfig(absoluteEncoderOffset));

    turnRelativeEncoder = turnSparkMax.getEncoder();
    turnPIDController = turnSparkMax.getPIDController();

    driveEncoder = driveSparkMax.getEncoder();
    drivePIDController = driveSparkMax.getPIDController();

    configDrive();
    configTurn();

    drivePositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(driveEncoder::getPosition);
    turnPositionQueue = SparkMaxOdometryThread.getInstance().registerSignal(turnRelativeEncoder::getPosition);

    driveFF = new SimpleMotorFeedforward(0, 0, 0);
    turnFF = new SimpleMotorFeedforward(0, 0, 0);
  }

  @Override
  public void updateInputs(ModuleIOInputs inputs) {
    inputs.drivePositionMeters = driveEncoder.getPosition();
    inputs.driveVelocityMetersPerSec = driveEncoder.getVelocity();
    inputs.driveAppliedVolts = driveSparkMax.getAppliedOutput() * driveSparkMax.getBusVoltage();
    inputs.driveDesiredMetersPerSec = driveSetpointMetersPerSec;
    inputs.driveCurrentAmps = new double[] {driveSparkMax.getOutputCurrent()};

    inputs.turnAbsolutePosition = Rotation2d.fromDegrees(turnAbsoluteEncoder.getAbsolutePosition().asSupplier().get());
    inputs.turnPosition = Rotation2d.fromDegrees(turnRelativeEncoder.getPosition());
    inputs.turnVelocityRadPerSec = turnRelativeEncoder.getVelocity();
    inputs.turnAppliedVolts = turnSparkMax.getAppliedOutput() * turnSparkMax.getBusVoltage();
    inputs.turnDesiredPosition = Rotation2d.fromDegrees(angleSetpointDegrees);
    inputs.turnCurrentAmps = new double[] {turnSparkMax.getOutputCurrent()};

    inputs.odometryDrivePositionsRad =
        drivePositionQueue.stream()
            .mapToDouble((Double value) -> value)
            .toArray();
    inputs.odometryTurnPositions =
        turnPositionQueue.stream()
            .map((Double value) -> value)
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
    drivePIDController.setReference(
      metersPerSecond, 
      ControlType.kVelocity,
      0,
      driveFF.calculate(metersPerSecond),
      ArbFFUnits.kVoltage
    );
  }

  @Override
  public void setPosition(double degrees) {
    angleSetpointDegrees = degrees;
    turnPIDController.setReference(
      degrees, 
      ControlType.kPosition,
      0,
      // Uses velocity input as direction, only for kS tuning
      turnFF.calculate(
        Math.signum(
          degrees - 
          turnRelativeEncoder.getPosition() ) ),
      ArbFFUnits.kVoltage );
  }

  public void configDrive() {
    driveSparkMax.restoreFactoryDefaults();
    driveSparkMax.setCANTimeout(250);
    driveSparkMax.setSmartCurrentLimit(40);
    driveSparkMax.enableVoltageCompensation(12.0);
    driveSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 4);
    setDriveBrakeMode(true);

    driveEncoder.setPosition(0.0);
    driveEncoder.setMeasurementPeriod(10);
    driveEncoder.setAverageDepth(2);
    driveEncoder.setPositionConversionFactor( DriveConstants.kDriveMetersCF);
    driveEncoder.setVelocityConversionFactor( DriveConstants.kDriveMetersVelocityCF );

    drivePIDController.setP(1.0, 0);
    drivePIDController.setI(0.0, 0);
    drivePIDController.setD(0.0, 0);
    drivePIDController.setFeedbackDevice(driveEncoder);

    driveSparkMax.burnFlash();
  }

  public void configTurn() {
    turnSparkMax.restoreFactoryDefaults();

    turnSparkMax.setInverted(DriveConstants.kTurnMotorInvert);
    turnSparkMax.setCANTimeout(250);
    turnSparkMax.setSmartCurrentLimit(30);
    turnSparkMax.enableVoltageCompensation(12.0);
    turnSparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 4);
    setTurnBrakeMode(true);

    turnRelativeEncoder.setPosition( turnAbsoluteEncoder.getAbsolutePosition().getValueAsDouble() );
    turnRelativeEncoder.setMeasurementPeriod(10);
    turnRelativeEncoder.setAverageDepth(2);
    turnRelativeEncoder.setPositionConversionFactor( DriveConstants.kTurnDegreesCF);
    turnRelativeEncoder.setVelocityConversionFactor( DriveConstants.kTurnDegreesVelocityCF );

    turnPIDController.setP(1.0);
    turnPIDController.setI(0.0);
    turnPIDController.setD(0.0);
    turnPIDController.setPositionPIDWrappingEnabled(true);
    turnPIDController.setPositionPIDWrappingMinInput(0);
    turnPIDController.setPositionPIDWrappingMaxInput(360);
    turnPIDController.setFeedbackDevice(turnRelativeEncoder);

    turnSparkMax.burnFlash();
  }
}
