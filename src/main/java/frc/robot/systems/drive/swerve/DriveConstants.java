package frc.robot.systems.drive.swerve;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.geometry.Rotation2d;

public class DriveConstants {
  public static double kWheelRadius = 0.1016;
  public static double kMaxSpeed = 4.5;

  public static final double kDriveGearRatio = (50.0 / 14.0) * (17.0 / 27.0) * (45.0 / 15.0);
  public static final double kTurnGearRatio = 150.0 / 7.0;

  public static final double kDriveMetersCF = 
    2 * Math.PI * DriveConstants.kWheelRadius / (DriveConstants.kDriveGearRatio);
  public static final double kDriveMetersVelocityCF = 
    2 * Math.PI * DriveConstants.kWheelRadius / (60 * DriveConstants.kDriveGearRatio);

  public static final boolean kTurnMotorInvert = true;
  public static final double kTurnDegreesCF = 
    2 * Math.PI * DriveConstants.kWheelRadius / (DriveConstants.kTurnGearRatio);
  public static final double kTurnDegreesVelocityCF = 
    2 * Math.PI * DriveConstants.kWheelRadius / (60 * DriveConstants.kTurnGearRatio);


  public static CANcoderConfiguration getCANCoderConfig(Rotation2d offset) {
    CANcoderConfiguration canCoderConfig = new CANcoderConfiguration();
    canCoderConfig.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Unsigned_0To1;
    canCoderConfig.MagnetSensor.SensorDirection =
        (!kTurnMotorInvert)
            ? SensorDirectionValue.Clockwise_Positive
            : SensorDirectionValue.CounterClockwise_Positive;
    canCoderConfig.MagnetSensor.MagnetOffset = offset.getDegrees();

    return canCoderConfig;
  }
}
