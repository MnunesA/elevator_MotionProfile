// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.constants;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {

  // GENERALS
  public static final double
      // Wheel Circumference = 2 * π * Diameter / 2
      CIRCUMFERENCE_WHEEL = (2 * 3.1415 * (0.0254 * 6) / 2), // meters
      DEADBAND_VALUE = 0.04;


  // CONTROLLERS JOYSTICK
  public static final int 
      XBOXCONTROL_ID = 0,
      JOYSTICK_ID = 1;

  // DIO Channels
  public static final int
      CHANNEL_DIO_0 = 0,
      CHANNEL_DIO_1 = 1,
      CHANNEL_DIO_2 = 2,
      CHANNEL_DIO_3 = 3,
      CHANNEL_DIO_4 = 4,
      CHANNEL_DIO_5 = 5,
      CHANNEL_DIO_6 = 6,
      CHANNEL_DIO_7 = 7,
      CHANNEL_DIO_8 = 8,
      CHANNEL_DIO_9 = 9;
      
  // PWM Channels
  public static final int
      CHANNEL_PWM_0 = 0,
      CHANNEL_PWM_1 = 1,
      CHANNEL_PWM_2 = 2,
      CHANNEL_PWM_3 = 3,
      CHANNEL_PWM_4 = 4,
      CHANNEL_PWM_5 = 5,
      CHANNEL_PWM_6 = 6,
      CHANNEL_PWM_7 = 7,
      CHANNEL_PWM_8 = 8,
      CHANNEL_PWM_9 = 9;
  
  
  // CAN Channels
  public static final int
      CHANNEL_CAN_0 = 0,
      CHANNEL_CAN_1 = 1,
      CHANNEL_CAN_2 = 2,
      CHANNEL_CAN_3 = 3,
      CHANNEL_CAN_4 = 4,
      CHANNEL_CAN_5 = 5;
  
  // PCM Channels
  public static final int
      CHANNEL_PCM_0 = 0,
      CHANNEL_PCM_1 = 1,
      CHANNEL_PCM_2 = 2,
      CHANNEL_PCM_3 = 3,
      CHANNEL_PCM_4 = 4,
      CHANNEL_PCM_5 = 5,
      CHANNEL_PCM_6 = 6,
      CHANNEL_PCM_7 = 7;
      
  // ANALOGGYRO's Channels
  public static final int
      CHANNEL_AGY_0 = 0,
      CHANNEL_AGY_1 = 1;
      
  // Xbox Control buttons
  public static final int
      XBOX_A_ID = 0,
      XBOX_B_ID = 0,
      XBOX_X_ID = 0,
      XBOX_Y_ID = 0,
      XBOX_VIEWBUTTON_ID = 0,
      XBOX_MENU_ID = 0,
      XBOX_LB_ID = 0,
      XBOX_RB_ID = 0;
}


