/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * The RobotMap is a mapping from the ports sensors and actuators are wired into
 * to a variable name. This provides flexibility changing wiring, makes checking
 * the wiring easier and significantly reduces the number of magic numbers
 * floating around.
 */
public class RobotMap {
  // These are PIO ids

  //Encoder 1
  public static final int EncoderPIOId1 = 5;
  public static final int EncoderPIOId2 = 6;  

  /* Below added by Ian
  * I added these to give the capability to test multiple encoders or switch out which encoders
  * we are testing
  */

  //Encoder 2
  public static final int EncoderPIOId3 = 7; 
  public static final int EncoderPIOId4 = 8;
 

  /* End of Edit */

  public static final int LimitSwitchPIOId1 = 0;
  public static final int LimitSwitchPIOId2 = 1;
  public static final int LimitSwitchPIOId3 = 2;
  
  // these are PWM ids
  public static final int SparkMotorId1 = 5;
  public static final int SparkMotorId2 = 6;

  // these are CAN bus ids
  public static final int TalonMotorCanId1 = 8;
  public static final int TalonMotorCanId2 = 7;
  public static final int TalonMotorCanID3 = 6;
  public static final int TalonMotorCanID4 = 5;
  public static final double TalonMinOutput = -0.5;
  public static final double TalonMaxOutput = 0.5;

  public static final double TalonPID_P = 4; //0.2;
  public static final double TalonPID_I = 0.0;
  public static final double TalonPID_D = 0.0;
  public static final double TalonPID_F = 0.0;
  public static final int    TalonCruiseSpeed = 15000;
  public static final int    TalonAcceleration = 6000;
  public static final double TalonAbsTolerance = 5;
  public static final double TalonUpSpeed = -.3;
  public static final double TalonDownSpeed = .3;

  // test elevator values
  public static final int SparkElevator = 5;
  public static final double ElevatorUpSpeed = -.4;
  public static final double ElevatorDownSpeed = .4;

  // Joystick to use
  public static final int JoystickId = 0;

  // these are controller button ids (on joystick)
  public static final int JoystickButtonA = 1;
  public static final int JoystickButtonB = 2;
  public static final int JoystickButtonX = 3;
  public static final int JoystickButtonY = 4;
  public static final int JoystickButtonShoulderLeft = 5;
  public static final int JoystickButtonShoulderRight = 6;


  // joystick axis mapping
  public static final int XboxController_Speed = 0;
  public static final int XboxController_Rotation = 1;
  
  public static final double JoystickDeadZone = 0.05;
}
