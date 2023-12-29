package frc.robot.systems.arm.singlejointed;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class SingleJointed extends SubsystemBase {

  private final SingleJointedIO m_kArmIO;

  private final SingleJointedIOInputsAutoLogged m_kInputs = new SingleJointedIOInputsAutoLogged();

  public SingleJointed(SingleJointedIO armIO) {
    m_kArmIO = armIO;
  }

  @Override
  public void periodic() {
    m_kArmIO.updateInputs(m_kInputs);
  }
}
