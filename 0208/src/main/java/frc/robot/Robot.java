// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.CounterBase.EncodingType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.ctre.phoenix.motorcontrol.can.VictorSPX;
import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.InvertType;


public class Robot extends TimedRobot {

  private VictorSPX RFmotor = new VictorSPX(2);
  private VictorSPX LFmotor = new VictorSPX(3);
  private TalonFX RBmotor = new TalonFX(0);
  private TalonFX LBmotor = new TalonFX(9);
  
  private XboxController joy1 = new XboxController(0);
  
  private Encoder encoder = new Encoder(0,1,true,EncodingType.k4X);       

  private final double kDriveTick2Feet = 1.0 / 4096 * 13.5 * Math.PI ; 
  final double kP = 0.002;  
  final double kI = 0.05;
  final double ilimit = 100;

  double setpoint = 0;
  double errorsum = 0;
  double lasttimestamp = 0;

  double error = 0;
  double dt = Timer.getFPGATimestamp() - lasttimestamp;

  @Override
  public void robotInit() {
    setpoint = 500;//新的
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {}

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    RBmotor.configFactoryDefault();
    RFmotor.configFactoryDefault();
    LBmotor.configFactoryDefault();
    LFmotor.configFactoryDefault();
    RBmotor.configPeakOutputForward(0.5); 
    RFmotor.configPeakOutputForward(0.5);    
    LBmotor.configPeakOutputForward(0.5); 
    LFmotor.configPeakOutputForward(0.5); 

    errorsum = 0;
    RBmotor.configPeakOutputReverse(-0.5); 
    RFmotor.configPeakOutputReverse(-0.5);    
    LBmotor.configPeakOutputReverse(-0.5); 
    LFmotor.configPeakOutputReverse(-0.5); 
    lasttimestamp=Timer.getFPGATimestamp(); 
    
    
    RBmotor.follow(LBmotor);
    RFmotor.follow(LFmotor);
    RBmotor.setInverted(InvertType.OpposeMaster);
    RFmotor.setInverted(InvertType.OpposeMaster);
  }

  @Override
  public void teleopPeriodic() {
    if (joy1.getRawButton(1)){
      setpoint = 50;
    }else 
    if (joy1.getRawButton(2)){
     setpoint = 0;
    }

    double sensorPosition = encoder.get() * kDriveTick2Feet; 
    error = setpoint - sensorPosition;
  

    if (Math.abs(error) < ilimit){
      errorsum += error * dt;
    }
    SmartDashboard.putNumber("encoder value", encoder.get() * kDriveTick2Feet);

    double outputSpeed = kP * error + kI * errorsum;
    
    LFmotor.set(ControlMode.PercentOutput,outputSpeed);
    LBmotor.set(ControlMode.PercentOutput,outputSpeed);
    lasttimestamp = Timer.getFPGATimestamp();

  SmartDashboard.putNumber("sensorposition", sensorPosition);
  SmartDashboard.putNumber("speed", outputSpeed);
  SmartDashboard.putNumber("percentOutput", LFmotor.getMotorOutputPercent());
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}
}
