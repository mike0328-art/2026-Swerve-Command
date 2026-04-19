// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;;

public class SwerveModule extends SubsystemBase {
  /** Creates a new SwerveModule. */
  private final TalonFX turningMotor;
  private final TalonFX driveMotor;
  //轉向馬達和移動馬達的物件
  private final TalonFXConfiguration turningConfig;
  private final TalonFXConfiguration driveConfig;
  //轉向馬達和移動馬達的設定
  private final CANcoder absolutedEncoder;
  private final CANcoderConfiguration cancoderConfig;
  //CANcoder的物件和設定
  private final PIDController turningPidController;
  private final SimpleMotorFeedforward driveFeedForward;
  //PIDcontroller和移動的Feedforward
  public SwerveModule(int turningMotor_ID, int driveMotor_ID, int absolutedEncoder_ID, double offset) {
    //建立Swerve的物件和設定的地方
    turningMotor = new TalonFX(turningMotor_ID);
    driveMotor = new TalonFX(driveMotor_ID);
    //建立轉向馬達和移動馬達的物件
    turningConfig = new TalonFXConfiguration();
    driveConfig = new TalonFXConfiguration();
    //建立轉向馬達和移動馬達的設定
    absolutedEncoder = new CANcoder(absolutedEncoder_ID);
    cancoderConfig = new CANcoderConfiguration();
    //建立CANcoder的物件和設定
    turningPidController = new PIDController(ModuleConstants.turningPidController_Kp, ModuleConstants.turningPidController_Ki, ModuleConstants.turningPidController_Kd);
    turningPidController.enableContinuousInput(ModuleConstants.pidRangeMin, ModuleConstants.pidRangeMax);
    //還有PidController的P,I,D限制他的最大值跟最小值
    driveFeedForward = new SimpleMotorFeedforward(ModuleConstants.driveFeedforward_Ks, ModuleConstants.driveFeedforward_Kv);
    //設定Feedforward的ks,kv
    turningConfig.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;//順時針是正，逆時針是負
    driveConfig.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;//順時針是正，逆時針是負
    
    cancoderConfig.MagnetSensor.SensorDirection = SensorDirectionValue.CounterClockwise_Positive;//順時針是正，逆時針是負
    // cancoderConfig.MagnetSensor.AbsoluteSensorDiscontinuityPoint = AbsoluteSensorDiscontinuityPoint.Unsigned_0To1;
    cancoderConfig.MagnetSensor.MagnetOffset = offset;//幫你把轉數轉成角度，並且把0度對準你想要的方向

    turningMotor.getConfigurator().apply(turningConfig);
    driveMotor.getConfigurator().apply(driveConfig);
    absolutedEncoder.getConfigurator().apply(cancoderConfig);
    //把設定各別放進對應的馬達裡

    turningMotor.setNeutralMode(NeutralModeValue.Brake);
    driveMotor.setNeutralMode(NeutralModeValue.Brake);
    //在平常沒有動時是會有煞車的
    resetEncoder();//把馬達的轉數歸零，這樣才不會有誤差值
  }

  public void resetEncoder() {
    driveMotor.setPosition(0);
  }//這一樣就是歸零

  public SwerveModuleState getState() {
    return new SwerveModuleState(getDriveVelocity(), Rotation2d.fromDegrees(getTurningAngle()));
  }//此時機器的速度和輪子角度

  public SwerveModulePosition getPosition() {
    return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromDegrees(getTurningAngle()));
  }//此時機器的距離和輪子角度

  public double getDriveVelocity() {
    return driveMotor.getVelocity().getValueAsDouble()*ModuleConstants.driveEncoderRot2Meter;
  }
  //從馬達的速度轉換成實際的速度，乘上常數

  public double getDrivePosition() {
    return driveMotor.getPosition().getValueAsDouble()*ModuleConstants.driveEncoderRot2Meter;
  }//從馬達的轉數轉換成實際的距離，乘上常數

  public double getTurningPosition() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble();
  }//從CANcoder的轉數轉換成實際的角度，乘上常數

  public double getTurningMotorPosition(){
    return turningMotor.getPosition().getValueAsDouble();
  }//轉向馬達的轉數轉換成實際的角度，乘上常數

  public double getTurningAngle() {
    return absolutedEncoder.getAbsolutePosition().getValueAsDouble()*360;
  }//從CANcoder的1到0，乘上360度

  public void stopMotor() {
    driveMotor.set(0);
    turningMotor.set(0);
  }//停止馬達

  public void setState(SwerveModuleState state) {
    // Turn Motor
    //把輪子轉到一個指定的角度，這裡會自動最短的角度轉過去
    state.optimize(getState().angle);
    double turningMotorOutput = turningPidController.calculate(getState().angle.getDegrees(), state.angle.getDegrees());
    turningMotor.set(turningMotorOutput);
    // Drive motor
    //例如我往前突然要往後輪子不會轉一圈再再往前而是直接反轉
    double driveMotorOutput = driveFeedForward.calculate(state.speedMetersPerSecond)/12;
    driveMotor.set(driveMotorOutput);//Feedforward的輸出是電壓，除以12
  }



  @Override
  public void periodic() {
  }
}
