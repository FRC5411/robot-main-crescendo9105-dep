package frc.robot.systems.arm.singlejointed;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.ProfiledPIDCommand;
import edu.wpi.first.wpilibj.Encoder;

public class SingleJointedSparkMaxIO implements SingleJointedIO {
  private final CANSparkMax armMotor;
  private final ProfiledPIDController pid;
  private final Encoder boreEncoder;  

  public SingleJointedSparkMaxIO(int port, int channelA, int channelB){
    armMotor = new CANSparkMax(port, MotorType.kBrushless);
    initializeMotors();
    pid = new ProfiledPIDController(0, 0, 0, null);
    boreEncoder = new Encoder(channelA, channelB);
  }

  public void moveToAngle(double angle){
    double output = pid.calculate(getRadians(), angle);
    setArm(output);
  }

  public double getRadians(){
    //find value to convert encoder value to degrees
    return boreEncoder.get() * Math.PI * 2;
  }

  public void setArm(double speed){
    armMotor.set(speed);
  }

  public void initializeMotors(){
    //Factory Reset
    armMotor.restoreFactoryDefaults();
    armMotor.setIdleMode(IdleMode.kBrake);
    armMotor.clearFaults();

    //Limits
    armMotor.setSmartCurrentLimit(60);
    // armMotor.setSoftLimit(SoftLimitDirection.kForward, (put limit here)) Constants not tuned yet
    // armMotor.setSoftLimit(SoftLimitDirection.kReverse, (put limit here)) Constants not tuned yet

  }
}
