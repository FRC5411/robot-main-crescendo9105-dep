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

package frc.robot.commands.tank;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import java.util.function.Consumer;
import java.util.function.Supplier;

public class TankDriveCommand extends Command {
  private double kP = 0;
  private double kI = 0;
  private double kD = 0;
  private double maxVelocity = 0;
  private double maxAcceleration = 0;

  public TankDriveCommand(
      Subsystem subsystem, Consumer<Double> voltageConsumer, Supplier<Double> velocitySupplier) {
    addRequirements(subsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    kP = 0;
    double kI = 0;
    double kD = 0;
    double maxVelocity = 0;
    double maxAcceleration = 0;
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}

/*
    public FunctionalCommand setVelocity(TankDrive tankDrive, double leftTargetMPS, double rightTargetMPS){
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double maxVelocity = 0;
        double maxAcceleration = 0;

        ProfiledPIDController leftPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        ProfiledPIDController rightPID = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));

        leftPID.setTolerance(2);
        rightPID.setTolerance(2);

        return new FunctionalCommand(
            // On Init
            () -> {
                leftPID.reset(m_kInputs.leftFrontVelocityMPS);
                rightPID.reset(m_kInputs.rightFrontVelocityMPS);
                System.out.println("Command SET VELOCITY has STARTED");
            },

            // On Execute
            () -> {
                double leftPIDCalc  = leftPID.calculate(m_kInputs.leftFrontVelocityMPS, leftTargetMPS);
                double rightPIDCalc = rightPID.calculate(m_kInputs.rightFrontVelocityMPS, rightTargetMPS);
                m_kDriveIO.setVolts(leftPIDCalc, rightPIDCalc);
            },

            // If Interrupted
            interrupted -> {
                System.out.println("Command SET VELOCITY has ENDED");
            },

            // Is Finished
            () -> leftPID.atGoal() && rightPID.atGoal(),

            // Requirements
            tankDrive
        );
    }
*/
