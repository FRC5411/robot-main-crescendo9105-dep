
package frc.robot.systems.drive.tank;

import org.littletonrobotics.junction.AutoLog;

import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.math.geometry.Pose2d;

public interface TankDriveIO {
    @AutoLog
    public static class TankDriveIOInputs {
        public double leftFrontPositionM = 0.0;
        public double leftBackPositionM = 0.0;
        public double rightFrontPositionM = 0.0;
        public double rightBackPositionM = 0.0;
    
        public double leftFrontVelocityMPS = 0.0;
        public double leftBackVelocityMPS = 0.0;
        public double rightFrontVelocityMPS = 0.0;
        public double rightBackVelocityMPS = 0.0;
    
        public double leftFrontTemperatureC = 0.0;
        public double leftBackTemperatureC = 0.0;
        public double rightFrontTemperatureC = 0.0;
        public double rightBackTemperatureC = 0.0;
    
        public double leftFrontAppliedCurrentAMP = 0.0;
        public double leftBackAppliedCurrentAMP = 0.0;
        public double rightFrontAppliedCurrentAMP = 0.0;
        public double rightBackAppliedCurrentAMP = 0.0;

        public double yawDEG = 0.0;
        public double pitchDEG = 0.0;

        public Pose2d estimatedPosition = new Pose2d();
    }

    /** Update IO inputs, called periodically */
    public default void updateInputs(TankDriveIOInputs inputs) {}

    /** Set the voltage of both sides of the drivetrain */
    public default void setVolts(double leftVolts, double rightVolts) {}

    /** Sets the motor's idle mode (brake or coast) */
    public default void setIdleMode(IdleMode mode) {}
}