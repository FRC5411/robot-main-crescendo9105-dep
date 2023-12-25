package frc.robot.systems.drive.tank;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class TankDrive extends SubsystemBase {

  private final TankDriveIO m_kDriveIO;

  private final TankDriveIOInputsAutoLogged m_kInputs = new TankDriveIOInputsAutoLogged();

  public TankDrive(TankDriveIO driveIO) {
    m_kDriveIO = driveIO;
  }

  @Override
  public void periodic() {
    m_kDriveIO.updateInputs(m_kInputs);
  }
}
