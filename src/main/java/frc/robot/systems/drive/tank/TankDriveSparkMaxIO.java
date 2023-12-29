package frc.robot.systems.drive.tank;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import frc.robot.systems.drive.tank.TankDriveVars.Constants;

public class TankDriveSparkMaxIO implements TankDriveIO {

  private final CANSparkMax m_kLeftFrontMotor =
      new CANSparkMax(Constants.leftFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax m_kLeftBackMotor =
      new CANSparkMax(Constants.leftBackMotorID, MotorType.kBrushless);
  private final CANSparkMax m_kRightFrontMotor =
      new CANSparkMax(Constants.rightFrontMotorID, MotorType.kBrushless);
  private final CANSparkMax m_kRightBackMotor =
      new CANSparkMax(Constants.rightBackMotorID, MotorType.kBrushless);

  private final RelativeEncoder m_kLeftFrontEncoder = m_kLeftFrontMotor.getEncoder();
  private final RelativeEncoder m_kLeftBackEncoder = m_kLeftBackMotor.getEncoder();
  private final RelativeEncoder m_kRightFrontEncoder = m_kRightFrontMotor.getEncoder();
  private final RelativeEncoder m_kRightBackEncoder = m_kRightBackMotor.getEncoder();

  private final DifferentialDriveKinematics m_kDriveKinematics =
      new DifferentialDriveKinematics(21.0);

  private final SparkMaxPIDController m_kLeftFrontController = m_kLeftFrontMotor.getPIDController();
  private final SparkMaxPIDController m_kRightFrontController = m_kRightFrontMotor.getPIDController();

  DifferentialDrive dDrive = new DifferentialDrive(m_kRightFrontMotor, m_kLeftFrontMotor);

  private final DifferentialDrivePoseEstimator m_kDriveOdometry =
      new DifferentialDrivePoseEstimator(
          m_kDriveKinematics, new Rotation2d(), 0.0, 0.0, new Pose2d());

  private final Pigeon2 m_kGyro = new Pigeon2(1);

  private final DifferentialDrive m_kDrive =
      new DifferentialDrive(
          new MotorControllerGroup(m_kLeftFrontMotor, m_kLeftBackMotor),
          new MotorControllerGroup(m_kRightFrontMotor, m_kRightBackMotor));

  public TankDriveSparkMaxIO() {
    updateHardwareSettings(m_kLeftFrontMotor, m_kLeftFrontEncoder, true);
    updateHardwareSettings(m_kLeftBackMotor, m_kLeftBackEncoder, true);
    updateHardwareSettings(m_kRightFrontMotor, m_kRightFrontEncoder, false);
    updateHardwareSettings(m_kRightBackMotor, m_kRightBackEncoder, false);

    m_kGyro.reset();
  }

  public void updateInputs(TankDriveIOInputs inputs) {
    inputs.leftFrontPositionM = m_kLeftFrontEncoder.getPosition();
    inputs.leftBackPositionM = m_kLeftBackEncoder.getPosition();
    inputs.rightFrontPositionM = m_kRightFrontEncoder.getPosition();
    inputs.rightBackPositionM = m_kRightBackEncoder.getPosition();

    inputs.leftFrontVelocityMPS = m_kLeftFrontEncoder.getVelocity();
    inputs.leftBackVelocityMPS = m_kLeftBackEncoder.getVelocity();
    inputs.rightFrontVelocityMPS = m_kRightFrontEncoder.getVelocity();
    inputs.rightBackVelocityMPS = m_kRightBackEncoder.getVelocity();

    inputs.leftFrontTemperatureC = m_kLeftFrontMotor.getMotorTemperature();
    inputs.leftBackTemperatureC = m_kLeftBackMotor.getMotorTemperature();
    inputs.rightFrontTemperatureC = m_kRightFrontMotor.getMotorTemperature();
    inputs.rightBackTemperatureC = m_kRightBackMotor.getMotorTemperature();

    inputs.leftFrontAppliedCurrentAMP = m_kLeftFrontMotor.getOutputCurrent();
    inputs.leftBackAppliedCurrentAMP = m_kLeftBackMotor.getOutputCurrent();
    inputs.rightFrontAppliedCurrentAMP = m_kRightFrontMotor.getOutputCurrent();
    inputs.rightBackAppliedCurrentAMP = m_kRightBackMotor.getOutputCurrent();

    inputs.yawDEG = m_kGyro.getYaw().getValue();
    inputs.pitchDEG = m_kGyro.getPitch().getValueAsDouble();

    m_kDriveOdometry.update(
        Rotation2d.fromDegrees(inputs.yawDEG),
        inputs.leftFrontPositionM,
        inputs.rightFrontPositionM);

    inputs.estimatedPosition = m_kDriveOdometry.getEstimatedPosition();
  }

  public void setVolts(double leftVolts, double rightVolts) {
    final var kLeftClampedVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    final var kRightClampedVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    m_kLeftFrontMotor.setVoltage(kLeftClampedVolts);
    m_kLeftBackMotor.setVoltage(kLeftClampedVolts);
    m_kRightFrontMotor.setVoltage(kRightClampedVolts);
    m_kRightBackMotor.setVoltage(kRightClampedVolts);

    m_kDrive.feed();
  }

  public void setArcadeDrive(double speed, double rotation) {
    WheelSpeeds ws = DifferentialDrive.arcadeDriveIK(speed, rotation, Constants.squareInputs);
    m_kLeftFrontController.setReference(ws.left, ControlType.kVelocity);
    m_kRightFrontController.setReference(ws.right, ControlType.kVelocity);
  }

  public void updateHardwareSettings(CANSparkMax motor, RelativeEncoder encoder, boolean inverted) {
    motor.clearFaults();
    motor.restoreFactoryDefaults();
    setIdleMode(IdleMode.kBrake);
    motor.setSmartCurrentLimit(60);
    motor.setInverted(inverted);

    encoder.setPositionConversionFactor(1.0);
    encoder.setVelocityConversionFactor(1.0 / 60.0);

    motor.burnFlash();
  }

  public void setIdleMode(IdleMode mode) {
    m_kLeftFrontMotor.setIdleMode(mode);
    m_kLeftBackMotor.setIdleMode(mode);
    m_kRightFrontMotor.setIdleMode(mode);
    m_kRightBackMotor.setIdleMode(mode);
  }
}
