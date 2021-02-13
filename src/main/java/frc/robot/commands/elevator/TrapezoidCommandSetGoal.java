// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.elevator;

import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.ConstantsElevator;
import frc.robot.subsystems.TrapezoidElevatorSubsystem;

public class TrapezoidCommandSetGoal extends CommandBase {
  
  private TrapezoidElevatorSubsystem tes;
  private TrapezoidProfile.State goal;
  private boolean limitSwitchStatus_UP;
  private boolean limitSwitchStatus_DOWN;
  private double position;
  private double velocity;
  private double currentPosition;
  private double currentVelocity;
  private double prevVelocity = 0.0;
  private double currentAcceleration;

  public TrapezoidCommandSetGoal(TrapezoidElevatorSubsystem trapezoidProfileSubsystem, double position, double velocity) {
    tes = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = velocity;
    addRequirements(tes);
  }

  public TrapezoidCommandSetGoal(TrapezoidElevatorSubsystem trapezoidProfileSubsystem, double position) {
    tes = trapezoidProfileSubsystem;
    this.position = position;
    this.velocity = 0.0;
    addRequirements(tes);
  }

  @Override
  public void initialize() {
    limitSwitchStatus_UP = !tes.getLimitSwitch_UP().get();
    limitSwitchStatus_DOWN = !tes.getLimitSwitch_DOWN().get();
    goal = new TrapezoidProfile.State(position, velocity);
    tes.setGoal(goal);
  }

  @Override
  public void execute() {
    this.currentVelocity = tes.getEncoder().getRate();
    this.currentAcceleration = (currentVelocity - prevVelocity)/0.02;
    this.prevVelocity = currentVelocity;
    this.tes.calculateFeedforward(currentVelocity, currentAcceleration);
  }

  @Override
  public void end(boolean interrupted) {
    if (tes.getLimitSwitch_UP().get() == limitSwitchStatus_UP) {
      new TrapezoidCommandSetGoal(tes, ConstantsElevator.ENCODER_FINAL).schedule();

    } else if (tes.getLimitSwitch_DOWN().get() == limitSwitchStatus_DOWN){
      new TrapezoidCommandSetGoal(tes, ConstantsElevator.ENCODER_INITIAL).schedule();
      
    } else {
      tes.getElevatorMotorsGroup().set(0.0);
    }
  }

  @Override
  public boolean isFinished() {
    boolean stop = (currentPosition == goal.position) ||
    (tes.getLimitSwitch_UP().get() == limitSwitchStatus_UP) || 
    (tes.getLimitSwitch_DOWN().get() == limitSwitchStatus_DOWN);
    return stop;
  }
}
