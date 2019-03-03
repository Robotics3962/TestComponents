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

  // indicate what we want to test
  public static enum TestModule {
    LimitSwitch,Elevator,ElevatorPid,Wrist,Arm,Encoder,Custom
  }
  public static TestModule testModule = TestModule.Elevator;//change to whatever you want to test meaning (TestModule.Elevator)

  // These are PIO ids

  //Encoder 1
  public static final int EncoderPIOId1 = 5;
  public static final int EncoderPIOId2 = 6;  

  public static final int ElevatorEncoderPIOId1 = 5;
  public static final int ElevatorEncoderPIOId2 = 6;  
  
  public static final int LimitSwitchPIOId1 = 0;
  public static final int LimitSwitchPIOId2 = 1;
  
  public static final int ElevatorTopLimitSwitchId = 0;
  public static final int ElevatorBottomLimitSwitchId = 1;

  public static final int ArmTopLimitSwitchId = 2;
  public static final int ArmBottomLimitSwitchId = 3;

  public static final int WristTopLimitSwitchId = 4;
  public static final int WristBottomLimitSwitchId = 5;
  
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

  /// generic
  public static final double TalonUpSpeed = -.3;
  public static final double TalonUpPidDelta = -20; // match sign of upspeed
  public static final double TalonDownSpeed = .3;
  public static final double TalonDownPidDelta = 20; // match sign of down speed
  public static final double TalonStopSpeed = .001;
  public static final double TalonAbsTolerance = 5;
  public static final int    TalonCruiseSpeed = 15000;
  public static final int    TalonAcceleration = 6000;
  public static final double TalonPID_P = 4; //0.2;
  public static final double TalonPID_I = 0.0;
  public static final double TalonPID_D = 0.0;
  public static final double TalonPID_F = 0.0;


  //arm
  public static final double TalonArmUpSpeed = .2;
  public static final double TalonArmUpPidDelta = 20; // match sign of upspeed
  public static final double TalonArmDownSpeed = -.2;
  public static final double TalonArmDownPidDelta = -20; // match sign of down speed
  public static final double TalonSrmStopSpeed = .001;
  public static final double TalonArmAbsTolerance = 5;
  public static final int    TalonArmCruiseSpeed = 15000;
  public static final int    TalonArmAcceleration = 6000;
  public static final double TalonArmPID_P = 4; //0.2;
  public static final double TalonArmPID_I = 0.0;
  public static final double TalonArmPID_D = 0.0;
  public static final double TalonArmPID_F = 0.0;
  public static final double TalonArmMaxUpPosition = 500; // this needs empirically set
  public static final double TalonArmMaxDownPosition = 0; // this needs empirically set

  //wrist 
  public static final double TalonWristUpSpeed = -.6;
  public static final double TalonWristUpPidDelta = -20; // match sign of upspeed
  public static final double TalonWristDownSpeed = .6;
  public static final double TalonWristDownPidDelta = 20; // match sign of down speed
  public static final double TalonWristStopSpeed = .001;
  public static final double TalonWristAbsTolerance = 5;
  public static final int    TalonWristCruiseSpeed = 15000;
  public static final int    TalonWristAcceleration = 6000;
  public static final double TalonWristPID_P = 4; //0.2;
  public static final double TalonWristPID_I = 0.0;
  public static final double TalonWristPID_D = 0.0;
  public static final double TalonWristPID_F = 0.0;
  public static final double TalonWristMaxUpPosition = 0; // this needs empirically set
  public static final double TalonWristMaxDownPosition = -500; // this needs empirically set

  // test elevator values
  public static final int SparkElevator = 5;
  public static final double ElevatorUpSpeed = .4;
  public static final double ElevatorDownSpeed = -.4;
  public static final double ElevatorStopSpeed = .001;
  public static final double ElevatorPID_P = 0.001;
  public static final double ElevatorPID_I = 0.0;
  public static final double ElevatorPID_D = 0.0;
  public static final double ElevatorPID_F = 0.0;
  public static final double ElevatorDistPerPulse = 1;
  public static final boolean ElevatorReverseDirection = false;
  public static final double ElevatorMinOutput = -.5;
  public static final double ElevatorMaxOutput = .5;
  public static final double ElevatorAbsTolerance = 5;
  public static final double ElevatorDownPidDelta = -1;
  public static final double ElevatorUpPidDelta = 1;

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

  // this is used to make the encoder phase check less sensitive
  public static final double EncoderSlop = 1;
}
