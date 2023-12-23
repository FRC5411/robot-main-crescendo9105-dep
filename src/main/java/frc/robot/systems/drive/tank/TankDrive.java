package frc.robot.systems.drive.tank;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class TankDrive extends SubsystemBase {
    
    private final TankDriveIO m_kDriveIO;

    private final TankDriveIOInputsAutoLogged m_kInputs = new TankDriveIOInputsAutoLogged();

    public TankDrive(TankDriveIO driveIO) {
        m_kDriveIO = driveIO;
    }

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
    
    @Override
    public void periodic() {
        m_kDriveIO.updateInputs(m_kInputs);
    }
}