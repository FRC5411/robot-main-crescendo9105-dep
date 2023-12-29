package frc.robot.systems.arm.singlejointed;

import com.revrobotics.CANSparkMax.IdleMode;
import edu.wpi.first.math.geometry.Pose2d;
import org.littletonrobotics.junction.AutoLog;

public interface SingleJointedIO {
  @AutoLog
  public static class SingleJointedIOInputs {
    public double armPositionR = 0.0;
    
    public double armVelocityRPS = 0.0;
    
    public double armTemperatureC = 0.0;

    public double armAppliedCurrentAMP = 0.0;
  }

  /** Update IO inputs, called periodically */
  public default void updateInputs(SingleJointedIOInputs inputs) {}

  /** Set the voltage to the single jointed arm */
  public default void setVolts(double leftVolts, double rightVolts) {}

  /** Sets the motor's idle mode (brake or coast) */
  public default void setIdleMode(IdleMode mode) {}
}
