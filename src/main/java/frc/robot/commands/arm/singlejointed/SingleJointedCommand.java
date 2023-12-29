package frc.robot.commands.arm.singlejointed;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.systems.arm.singlejointed.SingleJointed;
import frc.robot.systems.arm.singlejointed.SingleJointedVars.Constants.Points;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class SingleJointedCommand extends Command {
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double maxVelocity = 0;
  private double maxAcceleration = 0;
  private double setpoint = 0;
  private SingleJointed singleJointedArm;
  private ProfiledPIDController pid;

  public SingleJointedCommand(SingleJointed singleJointedArm, Points point) {
    this.singleJointedArm = singleJointedArm;
    setSetpoint(point);
    addRequirements(singleJointedArm);
  }

  public void setSetpoint(Points point){
    switch (point){
      case LOW:
        this.setpoint = 0;
        break;
      case MEDIUM:
        this.setpoint = 0.5;
        break;
      case HIGH:
        this.setpoint = 1;
        break;
    }

  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    // TODO: TUNE ME!
    kP = 0;
    kI = 0;
    kD = 0;
    maxVelocity = 0;
    maxAcceleration = 0;

    // TODO: SET ALL THESE VALUES WITH AK
    // pid.reset(singleJointedArm.getRadians());
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    // TODO: SET ALL THESE VALUES WITH AK
    // pid.calculate(singleJointedArm.getRadians(), setpoint);
    // singleJointedArm.setArm(pid.getSetpoint());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
