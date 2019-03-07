/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.ElevatorUpCmd;
import frc.robot.commands.ElevatorDownCmd;
import frc.robot.commands.EncoderTestCmd;
import frc.robot.commands.LimitSwitchTestCmd;
import frc.robot.commands.ResetTalonEncoderCmd;
import frc.robot.commands.TalonArmPIDMove;
import frc.robot.commands.TalonWristMoveDownCmd;
import frc.robot.commands.TalonWristMoveUpCmd;
import frc.robot.commands.TalonWristPIDMove;
import frc.robot.commands.TalonArmMoveDownCmd;
import frc.robot.commands.TalonArmMoveUpCmd;
import frc.robot.commands.ResetArmEncoderCmd;
import frc.robot.commands.ResetWristEncoderCmd;
import frc.robot.commands.ResetElevatorEncoderCmd;
import frc.robot.commands.ElevatorPidMoveUpCmd;
import frc.robot.commands.ElevatorPidMoveCmd;
import frc.robot.commands.ElevatorPidMoveDownCmd;
import frc.robot.commands.ShootBallCmd;
import frc.robot.commands.GrabBallCmd;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.buttons.JoystickButton;

/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
  Joystick joystick = new Joystick(RobotMap.JoystickId); 

  public OI() {     
    JoystickButton buttonA = new JoystickButton(joystick, RobotMap.JoystickButtonA);
    JoystickButton buttonB = new JoystickButton(joystick, RobotMap.JoystickButtonB);
    JoystickButton buttonX = new JoystickButton(joystick, RobotMap.JoystickButtonX);
    JoystickButton buttonY = new JoystickButton(joystick, RobotMap.JoystickButtonY);
    JoystickButton buttonLS = new JoystickButton(joystick, RobotMap.JoystickButtonShoulderLeft);
    JoystickButton buttonRS = new JoystickButton(joystick, RobotMap.JoystickButtonShoulderRight);

    // map the buttons to whatever module we are testing
    switch(RobotMap.testModule){
      case Custom:
        // don't create any subsystems
        break;
      case LimitSwitch:
        // log the state of the limit switch
        buttonA.whileHeld(new LimitSwitchTestCmd());
        break;
      case Elevator:
        // reset the encoder (set it to 0)
        buttonA.whenPressed(new ResetElevatorEncoderCmd());
        // manually move the elevator and log the position of the encoder
        buttonRS.whileHeld(new ElevatorUpCmd());
        buttonLS.whileHeld(new ElevatorDownCmd());
        break;
      case Wrist:
        buttonA.whileHeld(new TalonWristPIDMove(100));
        buttonB.whileHeld(new TalonWristPIDMove(200));
        buttonY.whileHeld(new TalonWristPIDMove(300));
        buttonX.whenPressed(new ResetWristEncoderCmd());
        buttonRS.whileHeld(new TalonWristMoveUpCmd());
        buttonLS.whileHeld(new TalonWristMoveDownCmd());
        break;
      case Arm:
        buttonA.whileHeld(new TalonArmPIDMove(100));
        buttonB.whileHeld(new TalonArmPIDMove(200));
        buttonY.whileHeld(new TalonArmPIDMove(300));
        buttonX.whenPressed(new ResetArmEncoderCmd());
        buttonRS.whileHeld(new TalonArmMoveUpCmd());
        buttonLS.whileHeld(new TalonArmMoveDownCmd());
      break;
      case Encoder:
        buttonA.whileHeld(new EncoderTestCmd());
        break;
      case ElevatorPid:
        buttonX.whenPressed(new ResetElevatorEncoderCmd());
        buttonA.whenPressed(new ElevatorPidMoveDownCmd());
        buttonB.whenPressed(new ElevatorPidMoveUpCmd());
        buttonY.whenPressed(new ElevatorPidMoveCmd(10));
        break;
      case Intake:
        buttonA.whileHeld(new GrabBallCmd());
        buttonB.whileHeld(new ShootBallCmd());
    }
  }
}
