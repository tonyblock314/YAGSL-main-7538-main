package frc.robot.subsystems.climber;

import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

//import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ClimberConstants;

public class Climber extends SubsystemBase {

  TalonFX lefttalon = new TalonFX(ClimberConstants.LEFT_CLIMBER_CAN_ID);
  TalonFX righttalon = new TalonFX(ClimberConstants.RIGHT_CLIMBER_CAN_ID);

  TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
  MotorOutputConfigs brakeMode = new MotorOutputConfigs().withNeutralMode(NeutralModeValue.Brake);

  DutyCycleOut leftclimbOut = new DutyCycleOut(0);
  DutyCycleOut rightclimbOut = new DutyCycleOut(0);
  // talonFXConfiguration.MotorOutput.NeutralMode = NeutralModeValue.brake;

  public Climber() 
  {
      lefttalon.getConfigurator().apply(new TalonFXConfiguration());
      lefttalon.getConfigurator().apply(brakeMode);
      righttalon.getConfigurator().apply(new TalonFXConfiguration());
      righttalon.getConfigurator().apply(brakeMode);

  }

    // Pushes new speed to intake wheel motor
  public void setLeftClimberSpeed(double speed) {
    // Updates intake wheel speed on dashboard
    leftclimbOut.Output = speed;

    SmartDashboard.putNumber("LClimber Speed: ", speed);

    lefttalon.setControl(leftclimbOut);
  }

  public void setRightClimberSpeed(double speed) {
    rightclimbOut.Output = speed;

    SmartDashboard.putNumber("RClimber Speed: ", speed);
    
    righttalon.setControl(rightclimbOut);
  }
}
