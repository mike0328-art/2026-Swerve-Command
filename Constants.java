// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.util.Units;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final double kJoystickDeadBand = 0.1;
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
  }//手把的常數，死區和手把的ID

  public static double setMaxOutput(double output, double maxOutput){
    return Math.min(maxOutput, Math.max(-maxOutput, output));
  }//限制輸出在最大值和最小值之間

  public static class ModuleConstants {
    public static final double pidRangeMin = -180;
    public static final double pidRangeMax = 180;
   //PID的範圍範圍是-180到180
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    //輪子的直徑是4英吋轉成公尺
    public static final double driveGearRatio = 1/5.36;
    public static final double turningGearRatio = 1.0/(150/7);
   //前後左右和轉向馬達的齒輪比
    public static final double driveVelocityConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;
    //齒輪比的倒數乘輪子周長，把馬達的轉速轉換成底盤的速度
    public static final double drivePositionConversionFactor = 
    (1/driveGearRatio)*wheelDiameterMeters*Math.PI;//齒輪比的倒數乘輪子周長，把馬達的轉數轉成底盤的距離

    public static final double driveEncoderRot2MeterPerSec = driveGearRatio*Math.PI*wheelDiameterMeters;//齒輪比乘輪子周長，把馬達的轉速轉換成底盤的速度
    public static final double driveEncoderRot2Meter = driveGearRatio*Math.PI*wheelDiameterMeters;//齒輪比乘輪子周長，把馬達的轉數轉成底盤的距離
    public static final double turningEncoderRot2RadPerSec = turningGearRatio*2*Math.PI;//齒輪比乘2π，把轉向馬達的轉速轉換成底盤轉的角速度
    public static final double driveEncoderRot2MeterPerMin = driveEncoderRot2MeterPerSec*60;//把每秒的速度轉換成每分鐘的速度
    public static final double driveEncoderRot2RadPerMin = turningEncoderRot2RadPerSec*60;//把每秒的轉角速度轉換成每分鐘的轉角速度

    public static final double kModuleDistance = Units.inchesToMeters(22.24);//前後左右馬達的距離，從底盤中心到馬達的距離是22.24英吋轉成公尺

    public static SwerveDriveKinematics swerveKinematics = new SwerveDriveKinematics(
      new Translation2d(kModuleDistance/2, kModuleDistance/2),
      new Translation2d(kModuleDistance/2, -kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, kModuleDistance/2),
      new Translation2d(-kModuleDistance/2, -kModuleDistance/2)
    );//建立四顆馬達的位置，從底盤中心到馬達的距離是22.24英吋轉公尺

    public static final double turningPidController_Kp = 0.013;
    public static final double turningPidController_Ki = 0;
    public static final double turningPidController_Kd = 0.0001;
    //PIDcontroller的P,I,D值
    public static final double driveFeedforward_Ks = 0.13;
    public static final double driveFeedforward_Kv = 2;
     //Feedforward的ks和kv值
  }

  public class SwerveConstants {
    public static final int leftFrontDrive_ID = 2;
    public static final int leftBackDrive_ID = 1;
    public static final int rightFrontDrive_ID = 3;
    public static final int rightBackDrive_ID = 4;
    //前後左右馬達的ID
    public static final int leftFrontTurning_ID = 15;
    public static final int leftBackTurning_ID = 5;
    public static final int rightFrontTurning_ID = 16;
    public static final int rightBackTurning_ID = 17;
    //前後左右轉向馬達的ID
    public static final int leftFrontAbsolutedEncoder_ID = 42;
    public static final int leftBackAbsolutedEncoder_ID = 41;
    public static final int rightFrontAbsolutedEncoder_ID = 43;
    public static final int rightBackAbsolutedEncoder_ID = 44;
    //前後左右CANcoder的ID
    public static final double leftFrontOffset = -0.342285;
    public static final double leftBackOffset = 0.144287;
    public static final double rightFrontOffset = -0.27856;
    public static final double rightBackOffset = 0.155029;
    public static final int gyro_ID = 56;
    //前後左右CANcoder的offset
    public static final double wheelDiameterMeters = Units.inchesToMeters(4);
    //輪子的直徑是4英吋轉成公尺
    public static final double pathingMoving_Kp = 0;
    public static final double pathingMoving_Ki = 0;
    public static final double pathingMoving_Kd = 0;
    //路徑的PID值
    public static final double pathingtheta_Kp = 0;
    public static final double pathingtheta_Ki = 0;
    public static final double pathingtheta_Kd = 0;
    //路徑的轉向PID值
    public static final double maxOutput = 0;
    //限制輸出的最大值
    public static final double maxDriveSpeed_MeterPerSecond = 5.94;
    public static final double kDriveBaseRadius = 15.73 * 0.0254;
    public static final double maxAngularVelocity_Angle = 850;
    //底盤的最大速度和最大轉角速度
  }
}
