package frc.robot.subsystems.intake;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.IntakeConstants;

public class Intake extends SubsystemBase {
  TalonFX talon = new TalonFX(IntakeConstants.INTAKE_CAN_ID);
  TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  DutyCycleOut ringOut = new DutyCycleOut(0);

  public Intake() {
      talon.getConfigurator().apply(new TalonFXConfiguration());
  }

    // Pushes new speed to intake wheel motor
  public void setIntakeSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    ringOut.Output = speed;
    SmartDashboard.putNumber("Intake Speed: ", speed);
    talon.setControl(ringOut);
  }
}
