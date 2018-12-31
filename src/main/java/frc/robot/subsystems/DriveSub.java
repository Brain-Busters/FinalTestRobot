/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PWMVictorSPX;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.command.Subsystem;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap;
import frc.robot.commands.DriveCmd;
/**
 * Add your docs here.
 */
public class DriveSub extends Subsystem {
  // Put methods for controlling this subsystem
  // here. Call these from Commands.

  public double distanceRight;
  public double distanceLeft;
  public double rateRight;
  public double rateLeft;
  public double avgDistance;
  public static PWMVictorSPX leftOne = new PWMVictorSPX(RobotMap.left1);
  public static PWMVictorSPX leftTwo = new PWMVictorSPX(RobotMap.left2);
  public static SpeedControllerGroup Gleft = new SpeedControllerGroup(leftOne, leftTwo);


  public static PWMVictorSPX rightOne = new PWMVictorSPX(RobotMap.right1);
  public static PWMVictorSPX rightTwo = new PWMVictorSPX(RobotMap.right2);
  public static SpeedControllerGroup Gright = new SpeedControllerGroup(rightOne, rightTwo);
  public static Encoder m_encoderRight = new Encoder(RobotMap.rightEncoderPort1, RobotMap.rightEncoderPort2, false, Encoder.EncodingType.k4X);
	public static Encoder m_encoderLeft = new Encoder(RobotMap.leftEncoderPort1, RobotMap.leftEncoderPort2, true, Encoder.EncodingType.k4X);
  public static final DifferentialDrive DriveBase = new DifferentialDrive(Gleft, Gright);
  
  public void robotDriver(Joystick joystickZero){
    DriveBase.arcadeDrive(-joystickZero.getY(), -joystickZero.getX());
  }



  public void encoderInit(){
    m_encoderRight.setDistancePerPulse((Math.PI * 3.25) / 256);
		m_encoderRight.setMaxPeriod(.1);
		m_encoderRight.setMinRate(10);
		m_encoderRight.setReverseDirection(true);
		m_encoderRight.setSamplesToAverage(7);

		m_encoderLeft.setDistancePerPulse((Math.PI * 3) / 256);
		m_encoderLeft.setMaxPeriod(.1);
		m_encoderLeft.setMinRate(10);
		m_encoderLeft.setReverseDirection(false);
    m_encoderLeft.setSamplesToAverage(7);
  }



  public void encoderReset(){
    m_encoderRight.reset();
    m_encoderLeft.reset();
  }



  public void encoderUpdate(){
    distanceRight = m_encoderRight.getDistance();
    distanceLeft = m_encoderLeft.getDistance();
    rateRight = m_encoderRight.getRate();
    rateLeft = m_encoderLeft.getRate();
    avgDistance = (distanceLeft + distanceRight)/2;
    SmartDashboard.putNumber("Right Distance", distanceRight);
		SmartDashboard.putNumber("Left Distance", distanceLeft);
  }



  public void driveForward(double speed){
    DriveBase.arcadeDrive(speed, 0);
  }



  public void driveStop(){
    DriveBase.arcadeDrive(-.2, 0);
  }


  @Override
  public void initDefaultCommand() {
    // Set the default command for a subsystem here.
    setDefaultCommand(new DriveCmd());

  }
}
