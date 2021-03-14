// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.constants.Constants;
import frc.robot.constants.ConstantsElevator;
import frc.robot.subsystems.TrapezoidElevatorSubsystem;
import frc.robot.commands.elevator.TrapezoidCommandSetGoal;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  
  private TrapezoidElevatorSubsystem tElevatorSubsystem;
  private XboxController xboxController;
  private JoystickButton XBOX_A;
  private POVButton POV_BUTTON_90;
  private POVButton POV_BUTTON_270;
  
  public RobotContainer() {
    
    configureJoysticks();
    configureSubsystems();
    configureCommands();
    configureButtonBindings();
    configureButtonFunctions();
    configureConditionalButtonFuntions();
  }

  
  private void configureButtonBindings() {
    XBOX_A = new JoystickButton(xboxController, Constants.XBOX_A_ID);
    POV_BUTTON_90 = new POVButton(xboxController, 90);
    POV_BUTTON_270 = new POVButton(xboxController, 270);
  }

  private void configureButtonFunctions() {}           

  protected void configureConditionalButtonFuntions() {

    switch (tElevatorSubsystem.getEncoder().get()) {
      case ConstantsElevator.ENCODER_FINAL:
      XBOX_A.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                     ConstantsElevator.ENCODER_INITIAL));
      POV_BUTTON_270.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                            (ConstantsElevator.ENCODER_FINAL/2)));
      break;
      case ConstantsElevator.ENCODER_INITIAL:
      XBOX_A.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                     ConstantsElevator.ENCODER_FINAL));
      POV_BUTTON_90.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                           (ConstantsElevator.ENCODER_FINAL/2)));
      break;
      default:
      POV_BUTTON_90.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                            ConstantsElevator.ENCODER_FINAL));
      POV_BUTTON_270.whenPressed(new TrapezoidCommandSetGoal(tElevatorSubsystem, ConstantsElevator.DISTANCE_PER_PULSE * 
                                                             ConstantsElevator.ENCODER_INITIAL));                 
    }
  }

  private void configureSubsystems() {
    tElevatorSubsystem = new TrapezoidElevatorSubsystem(Constants.CHANNEL_PWM_0,
                                                        Constants.CHANNEL_PWM_1,
                                                        ConstantsElevator.encoderElevator,
                                                        ConstantsElevator.limitSwitch_UP,
                                                        ConstantsElevator.limitSwitch_DOWN);
  }

  private void configureCommands() {
  }

  private void configureJoysticks() {
    xboxController = new XboxController(Constants.JOYSTICK_ID);
  }

  public XboxController getXboxController() {
    return xboxController;
  }

  /*
  public Command getAutonomousCommand() {
    // An ExampleCommand will run in autonomous
    return m_autoCommand;
  }
  */
}
