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

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import org.littletonrobotics.junction.Logger;

public class Module {
  // private static final double WHEEL_RADIUS = Units.inchesToMeters(2.0);
  public static final double ODOMETRY_FREQUENCY = 250.0;

  private final ModuleIO io;
  private final ModuleIOInputsAutoLogged inputs = new ModuleIOInputsAutoLogged();
  private final int index;

  private Rotation2d angleSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Double speedSetpoint = null; // Setpoint for closed loop control, null for open loop
  private Rotation2d turnRelativeOffset = null; // Relative + Offset = Absolute
  private double lastPositionMeters = 0.0; // Used for delta calculation
  private SwerveModulePosition[] positionDeltas = new SwerveModulePosition[] {};

  public Module(ModuleIO io, int index) {
    this.io = io;
    this.index = index;

    // Switch constants based on mode (the physics simulator is treated as a
    // separate robot with different tuning)
    setBrakeMode(true);
  }

  /**
   * Update inputs without running the rest of the periodic logic. This is useful since these
   * updates need to be properly thread-locked.
   */
  public void updateInputs() {
    io.updateInputs(inputs);
  }

  public void periodic() {
    Logger.processInputs("Drive/Module" + Integer.toString(index), inputs);
    Logger.recordOutput(
        "Drive/Module" + Integer.toString(index) + "/CurrentVel", inputs.driveVelocityMetersPerSec);
    Logger.recordOutput(
        "Drive/Module" + Integer.toString(index) + "/CurrentPos", inputs.turnPosition.getRadians());

    // On first cycle, reset relative turn encoder
    // Wait until absolute angle is nonzero in case it wasn't initialized yet
    if (turnRelativeOffset == null && inputs.turnAbsolutePosition.getRadians() != 0.0)
      turnRelativeOffset = inputs.turnAbsolutePosition.minus(inputs.turnPosition);

    if (angleSetpoint != null) {
      io.setPosition(angleSetpoint.getRadians());

      // Only allowed if closed loop turn control is running
      if (speedSetpoint != null)
        // When the error is 90°, the velocity setpoint should be 0. As the wheel turns
        // towards the setpoint, its velocity should increase. This is achieved by
        // taking the component of the velocity in the direction of the setpoint.
        // io.setVelocity(
        //     Math.cos(
        //             Math.min(
        //                 Math.PI / 2,
        //                 Math.abs(
        //                     inputs.turnDesiredPosition.getRadians()
        //                         - inputs.turnPosition.getRadians())))
        //         * speedSetpoint);
        io.setVelocity(speedSetpoint);
    }

    // Calculate position deltas for odometry
    int deltaCount =
        Math.min(inputs.odometryDrivePositionsRad.length, inputs.odometryTurnPositions.length);
    positionDeltas = new SwerveModulePosition[deltaCount];
    for (int i = 0; i < deltaCount; i++) {
      double positionMeters = inputs.odometryDrivePositionsRad[i] * DriveConstants.kWheelRadius;
      Rotation2d angle =
          inputs.odometryTurnPositions[i].plus(
              turnRelativeOffset != null ? turnRelativeOffset : new Rotation2d());
      positionDeltas[i] = new SwerveModulePosition(positionMeters - lastPositionMeters, angle);
      lastPositionMeters = positionMeters;
    }
  }

  /** Runs the module with the specified setpoint state. Returns the optimized state. */
  public SwerveModuleState setDesiredState(SwerveModuleState state) {
    // Optimize state based on current angle
    // Controllers run in "periodic" when the setpoint is not null
    var optimizedState = SwerveModuleState.optimize(state, getAngle());

    // Update setpoints, controllers run in "periodic"
    speedSetpoint = optimizedState.speedMetersPerSecond;
    // angleSetpoint =
    //     Math.abs(speedSetpoint / DriveConstants.kMaxSpeed) < 0.1
    //         ? optimizedState.angle
    //         : inputs.turnPosition;
    angleSetpoint = optimizedState.angle;

    return optimizedState;
  }

  /** Runs the module with the specified voltage while controlling to zero degrees. */
  public void runCharacterization(double volts) {
    // Closed loop turn control
    angleSetpoint = new Rotation2d();

    // Open loop drive control
    io.setDriveVoltage(volts);
    speedSetpoint = null;
  }

  /** Disables all outputs to motors. */
  public void stop() {
    io.setTurnVoltage(0.0);
    io.setDriveVoltage(0.0);

    // Disable closed loop control for turn and drive
    angleSetpoint = null;
    speedSetpoint = null;
  }

  /** Sets whether brake mode is enabled. */
  public void setBrakeMode(boolean enabled) {
    io.setDriveBrakeMode(enabled);
    io.setTurnBrakeMode(enabled);
  }

  /** Returns the current turn angle of the module. */
  public Rotation2d getAngle() {
    if (turnRelativeOffset == null) return new Rotation2d();
    return inputs.turnPosition.plus(turnRelativeOffset);
  }

  /** Returns the current drive position of the module in meters. */
  public double getPositionMeters() {
    return inputs.drivePositionMeters;
  }

  /** Returns the current drive velocity of the module in meters per second. */
  public double getVelocityMetersPerSec() {
    return inputs.driveVelocityMetersPerSec;
  }

  /** Returns the module position (turn angle and drive position). */
  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getPositionMeters(), getAngle());
  }

  /** Returns the module state (turn angle and drive velocity). */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getVelocityMetersPerSec(), getAngle());
  }

  /** Returns the module position deltas received this cycle. */
  public SwerveModulePosition[] getPositionDeltas() {
    return positionDeltas;
  }

  /** Returns the drive velocity in radians/sec. */
  public double getCharacterizationVelocity() {
    return inputs.driveVelocityMetersPerSec;
  }
}
