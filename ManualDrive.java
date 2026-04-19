// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.SwerveSubsystem;

public class ManualDrive extends Command {
  /** Creates a new ManualDrive. */
  private final SwerveSubsystem m_SwerveSubsystem;

  private final DoubleSupplier xSpeedFunc;
  private final DoubleSupplier ySpeedFunc;
  private final DoubleSupplier zSpeedFunc;
  private final BooleanSupplier isSlowFunc;
  //從手把推的速度和慢速按鈕的輸入
  private final SlewRateLimiter xLimiter;
  private final SlewRateLimiter yLimiter;
  private final SlewRateLimiter zLimiter;
   //用來限制加速度的，讓底盤不會突然加速或減速
  private double xSpeed;
  private double ySpeed;
  private double zSpeed;
  private boolean isSlow;
    //實際的速度
  public ManualDrive(SwerveSubsystem swerveSubsystem, DoubleSupplier xSpeed, DoubleSupplier ySpeed, DoubleSupplier zSpeed, BooleanSupplier isSlow) {
    // Use addRequirements() here to declare subsystem dependencies.
    this.m_SwerveSubsystem = swerveSubsystem;
    this.xSpeedFunc = xSpeed;
    this.ySpeedFunc = ySpeed;
    this.zSpeedFunc = zSpeed;
    this.isSlowFunc = isSlow;
    //輸入的速度和慢速按鈕
    this.xLimiter = new SlewRateLimiter(4.6);
    this.yLimiter = new SlewRateLimiter(4.6);
    this.zLimiter = new SlewRateLimiter(4.6);
    //加速度限制，不然底盤會爆衝
    addRequirements(m_SwerveSubsystem);
    
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {

  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    this.xSpeed = -xSpeedFunc.getAsDouble();
    this.ySpeed = ySpeedFunc.getAsDouble();
    zSpeed = -zSpeedFunc.getAsDouble();
    //x和z要反過來，因為手把前推是負後推是正，旋轉也一樣
    this.xSpeed = MathUtil.applyDeadband(this.xSpeed, OperatorConstants.kJoystickDeadBand);
    this.ySpeed = MathUtil.applyDeadband(this.ySpeed, OperatorConstants.kJoystickDeadBand);
    this.zSpeed = MathUtil.applyDeadband(this.zSpeed, OperatorConstants.kJoystickDeadBand);
    //手把蘑菇頭有時會有輸出就算我沒推，所以要死區在超出死區這範圍內都不會有輸出
    this.xSpeed = xLimiter.calculate(this.xSpeed);
    this.ySpeed = yLimiter.calculate(this.ySpeed);
    this.zSpeed = zLimiter.calculate(this.zSpeed);
    //限制加速度，也是保護馬達跟機構
    this.isSlow = isSlowFunc.getAsBoolean();
    
    if(isSlow) {
      xSpeed = xSpeed*0.2;
      ySpeed = ySpeed*0.2;
      zSpeed = zSpeed*0.2;
    }else {
      xSpeed = xSpeed*0.8;
      ySpeed = ySpeed*0.8;
      zSpeed = zSpeed*0.8;
    }//慢速鍵按下去速度是原來的0.2，沒按是0.8

    SmartDashboard.putNumber("ManualDrive/Xspeed", xSpeed);
    SmartDashboard.putNumber("ManualDrive/Yspeed", ySpeed);
    SmartDashboard.putNumber("ManualDrive/Zspeed", zSpeed);
    //顯示實際的速度
    m_SwerveSubsystem.drive(this.xSpeed, this.ySpeed, zSpeed,true);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    m_SwerveSubsystem.drive(0, 0, 0, false);
  }//停止底盤

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
