// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ModuleConstants;
import frc.robot.Constants.SwerveConstants;;

public class SwerveSubsystem extends SubsystemBase {
  /** Creates a new SwerveSubsystem. */
  private final SwerveModule leftFront;
  private final SwerveModule leftBack;
  private final SwerveModule rightFront;
  private final SwerveModule rightBack;//四個馬達的物件
  
  private final Pigeon2 gyro;//陀螺儀的物件
  private final Pigeon2Configuration gyroConfig;//陀螺儀的設定
  //
  private final SwerveDriveOdometry odometry;//里程計的物件

  private final Field2d field;//在SmartDashboard上面標機器人位置

  private double zSpeed;//旋轉的速度
  
  /**
   * 
   */
  public SwerveSubsystem() {
    leftFront = new SwerveModule(
      SwerveConstants.leftFrontTurning_ID,
      SwerveConstants.leftFrontDrive_ID,
      SwerveConstants.leftFrontAbsolutedEncoder_ID,
      SwerveConstants.leftFrontOffset
            );
    rightFront = new SwerveModule(
      SwerveConstants.rightFrontTurning_ID,
      SwerveConstants.rightFrontDrive_ID,
      SwerveConstants.rightFrontAbsolutedEncoder_ID,
      SwerveConstants.rightFrontOffset);
    leftBack = new SwerveModule(
      SwerveConstants.leftBackTurning_ID,
      SwerveConstants.leftBackDrive_ID,
      SwerveConstants.leftBackAbsolutedEncoder_ID,
      SwerveConstants.leftBackOffset);
    rightBack = new SwerveModule(
      SwerveConstants.rightBackTurning_ID,
      SwerveConstants.rightBackDrive_ID,
      SwerveConstants.rightBackAbsolutedEncoder_ID,
      SwerveConstants.rightBackOffset);
      //四個底盤的物件，把對應的ID和offset丟進去
     gyro = new Pigeon2(SwerveConstants.gyro_ID);
     gyroConfig = new Pigeon2Configuration();
     //陀螺儀的物件和設定的地方
     gyroConfig.MountPose.MountPoseYaw = 0;
     gyroConfig.MountPose.MountPosePitch = 0;
     gyroConfig.MountPose.MountPoseRoll = 0;
      
     gyro.getConfigurator().apply(gyroConfig);
     //把設定丟進陀螺儀裡面
     field = new Field2d();
      //在SmartDashboard上面顯示機器人位置的物件
     odometry = new SwerveDriveOdometry(ModuleConstants.swerveKinematics, getRotation(), getModulesPosition(), getRobotPose());
      //里程計的物件
     resetGyro();
  }

  public ChassisSpeeds getChassisSpeed() {
    return ModuleConstants.swerveKinematics.toChassisSpeeds(getModuleStates());
  }//底盤的速度

  public Pose2d getRobotPose() {
    return field.getRobotPose();
  }//機器人位置

  public Rotation2d getRotation() {
    return gyro.getRotation2d();
  }//底盤旋轉的角度

  public SwerveModulePosition[] getModulesPosition() {
    return new SwerveModulePosition[]{
      leftFront.getPosition(),
      rightFront.getPosition(),
      leftBack.getPosition(),
      rightBack.getPosition()
    };
  }//四個馬達的位置

  public SwerveModuleState[] getModuleStates() {
    return new SwerveModuleState[]{
      leftFront.getState(),
      rightFront.getState(),
      leftBack.getState(),
      rightBack.getState()
    };//四個馬達的速度和角度
  }

  public void setModouleStates(SwerveModuleState[] desiredStates) {
      SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
      leftFront.setState(desiredStates[0]);
      rightFront.setState(desiredStates[1]);
      leftBack.setState(desiredStates[2]);
      rightBack.setState(desiredStates[3]);
  }//把四個馬達轉到指定的速度和角度，把速度限制在最大值以內

  public void setModouleStates_Auto(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConstants.maxDriveSpeed_MeterPerSecond);
    leftFront.setState(desiredStates[0]);
    rightFront.setState(desiredStates[1]);
    leftBack.setState(desiredStates[2]);
    rightBack.setState(desiredStates[3]);
}

  public void resetGyro() {
    gyro.reset();
  }//把陀螺儀歸零

  public void drive(double xSpeed, double ySpeed, double zSpeed, boolean fieldOrient) {
    SwerveModuleState[] state;//底盤的速度和角度
    xSpeed = xSpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    ySpeed = ySpeed * SwerveConstants.maxDriveSpeed_MeterPerSecond;
    zSpeed = zSpeed * Math.toRadians(SwerveConstants.maxAngularVelocity_Angle);//把輸入的速度從-1到1轉成實際的速度，乘最大速度
    this.zSpeed = zSpeed;
    if(fieldOrient) {
      state = ModuleConstants.swerveKinematics.toSwerveModuleStates(ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, zSpeed, getRotation()));
    }else{
      state = ModuleConstants.swerveKinematics.toSwerveModuleStates(new ChassisSpeeds(xSpeed, ySpeed, zSpeed));
    }
    setModouleStates(state);
  } 

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    odometry.update(getRotation(), getModulesPosition());
    field.setRobotPose(odometry.getPoseMeters());//里程和機器人位置

    SmartDashboard.putNumber("Swerve/leftFrontAbsolutePosion", leftFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/leftBackAbsolutePosion", leftBack.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightFrontAbsolutePosion", rightFront.getTurningPosition());
    SmartDashboard.putNumber("Swerve/rightBackAbsolutePosion", rightBack.getTurningPosition());
    //四個輪子的絕對角度，從CANcoder來的
    SmartDashboard.putNumber("Swerve/leftFrontTurningMotorPosition", leftFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/leftBackTurningMotorPosition", leftBack.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightFrontTurningMotorPosition", rightFront.getTurningMotorPosition());
    SmartDashboard.putNumber("Swerve/rightBackTurningMotorPosition", rightBack.getTurningMotorPosition());
    //四個輪子位置，從CANcoder來的
    SmartDashboard.putNumber("Swerve/leftFrontDrivingMotorPosition", leftFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/leftBackDrivingMotorPosition", leftBack.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightFrontDrivingMotorPosition", rightFront.getDrivePosition());
    SmartDashboard.putNumber("Swerve/rightBackDrivingMotorPosition", rightBack.getDrivePosition());
    SmartDashboard.putNumber("Swerve/zSpeed", zSpeed);
  }//這會顯示底盤在場地上的位置
}
