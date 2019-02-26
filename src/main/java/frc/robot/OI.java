/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.commands.EncoderTestCmd;
import frc.robot.commands.LimitSwitchTestCmd;
import frc.robot.commands.ResetTalonEncoderCmd;
import frc.robot.commands.TalonPIDMove;
import frc.robot.commands.TalonMoveDownCmd;
import frc.robot.commands.TalonMoveUpCmd;
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
    buttonA.whileHeld(new LimitSwitchTestCmd());

    JoystickButton buttonB = new JoystickButton(joystick, RobotMap.JoystickButtonB);
    buttonB.whileHeld(new EncoderTestCmd());

    JoystickButton buttonX = new JoystickButton(joystick, RobotMap.JoystickButtonX);
    buttonX.whenPressed(new ResetTalonEncoderCmd());

    JoystickButton buttonY = new JoystickButton(joystick, RobotMap.JoystickButtonY);
    buttonY.whenPressed(new TalonPIDMove(300));

    JoystickButton buttonLS = new JoystickButton(joystick, RobotMap.JoystickButtonShoulderLeft);
    buttonLS.whileHeld(new TalonMoveDownCmd());

    JoystickButton buttonRS = new JoystickButton(joystick, RobotMap.JoystickButtonShoulderRight);
    buttonRS.whileHeld(new TalonMoveUpCmd());

  }
}
