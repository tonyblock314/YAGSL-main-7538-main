package frc.robot.subsystems.shooter;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;

public class Shooter extends SubsystemBase {

  TalonFX lefttalon = new TalonFX(ShooterConstants.LEFT_SHOOTER_CAN_ID);
  TalonFX righttalon = new TalonFX(ShooterConstants.RIGHT_SHOOTER_CAN_ID);

  TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();

  DutyCycleOut leftringOut = new DutyCycleOut(0);
  DutyCycleOut rightringOut = new DutyCycleOut(0);

  public Shooter() 
  {
      lefttalon.getConfigurator().apply(new TalonFXConfiguration());
      righttalon.getConfigurator().apply(new TalonFXConfiguration());
  }

    // Pushes new speed to intake wheel motor
  public void setShooterSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    leftringOut.Output = speed;
    rightringOut.Output = -speed;

    SmartDashboard.putNumber("Intake Speed: ", speed);

    lefttalon.setControl(leftringOut);
    righttalon.setControl(rightringOut);
  }
}
