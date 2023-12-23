package frc.robot.systems.drive.tank;

import edu.wpi.first.wpilibj2.command.FunctionalCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.systems.drive.tank.TankDriveVars.Constants;
import frc.robot.systems.drive.tank.TankDriveVars.Vars;

public class TankDrive extends SubsystemBase {
    
    private final TankDriveIO m_kDriveIO;

    private final TankDriveIOInputsAutoLogged m_kInputs = new TankDriveIOInputsAutoLogged();

    public TankDrive(TankDriveIO driveIO) {
        m_kDriveIO = driveIO;
    }

    public FunctionalCommand setVelocity(TankDrive tankDrive, double targetMPS){
        double kP = 0;
        double kI = 0;
        double kD = 0;
        double maxVelocity = 0;
        double maxAcceleration = 0;

        ProfiledPIDController pid = new ProfiledPIDController(kP, kI, kD, new TrapezoidProfile.Constraints(maxVelocity, maxAcceleration));
        pid.setTolerance(2);
        
        return new FunctionalCommand(
            // On Init
            () -> {
                pid.reset(0);
                System.out.println("Command SET VELOCITY has STARTED");
            }, 
            
            // On Execute
            () -> {
                // LLLLLLLLLLLLLLLLLLLLLLLLLLLLLL
                double calc = pid.calculate(m_kInputs.leftFrontVelocityMPS, targetMPS);
                m_kDriveIO.setVelocityMPS(calc);
            }, 

            // If Interrupted
            interrupted -> {
                System.out.println("Command SET VELOCITY has ENDED");
            }, 

            // Is Finished
            () -> false, 
            
            // Requirements
            tankDrive
        );
    }
    
    @Override
    public void periodic() {
        m_kDriveIO.updateInputs(m_kInputs);
    }
}