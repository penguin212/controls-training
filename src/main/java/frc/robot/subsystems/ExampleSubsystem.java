// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.AnalogPotentiometer;
import edu.wpi.first.wpilibj.Timer;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.*;


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax launchmotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax launchmotor2 = new CANSparkMax(16, MotorType.kBrushless);//check if these should be opposite of each other  
  private final WPI_TalonSRX feedmotor = new WPI_TalonSRX(8);
  private final AnalogPotentiometer irbot = new AnalogPotentiometer(0);//at the bottom
  private final AnalogPotentiometer irmid = new AnalogPotentiometer(1);//at the middle
  private final AnalogPotentiometer irtop = new AnalogPotentiometer(3);//at the top
  private final XboxController controller = new XboxController(0);
  private final Timer t = new Timer();
  private final double BALL_DETECTED = .38;
  private final double BALL_DETECTED2 = .30;
  private final double BALL_DETECTED3 = .123;//SET THIS THING!!!!
  private final double FEED_SPEED = -.25;
  private final double LAUNCH_SPEED = -.25;
  private final double FIRING_TIME = 2;
  boolean bot = false;
  boolean mid = false;
  boolean top = false;
  boolean firing = false;
  boolean idle = true;
  int state = 0;
  double starttime;
  boolean active = false;
  
  public ExampleSubsystem() {
    launchmotor1.restoreFactoryDefaults();
    launchmotor2.restoreFactoryDefaults();
    feedmotor.configFactoryDefault();
    feedmotor.setNeutralMode(NeutralMode.Coast);
    launchmotor1.setIdleMode(IdleMode.kBrake);
    launchmotor2.setIdleMode(IdleMode.kBrake);
    launchmotor2.follow(launchmotor1); 
    
  }

  // public double getSpeed(double x){

  //   return(1 - x/.65);
  // }
  public int booltoint(boolean a){
    return a ? 1 : 0;
  }

  public void updateballs(){
    top = irtop.get() > BALL_DETECTED3;
    mid = irmid.get() > BALL_DETECTED2;
    bot = irbot.get() > BALL_DETECTED;
  }

  public void updatestate(){//possible states are 0,  10 11 12,  21 22 23,  32
    updateballs();
    int t = booltoint(top);
    int m = booltoint(mid);
    int b = booltoint(bot);
    state = 0;
    state += 10 * (t + m + b); //tens digit is number of balls in system
    if(top){ //ones digit is height of heighest ball, 23 is used for ball - nothing - ball case
      state += 2;
      if(!mid && bot){
        state += 1; //accounts for state 23
      }
      return;
    }
    if(mid){
      state++;
    }
    return;
  }

  @Override
  public void periodic() {
    if(controller.getAButtonPressed()){
      active = !active;
    }

    System.out.println(state + " " + irbot.get() + " " + irmid.get() + " " + irtop.get());


    if(!active){
      return; 
    }

    if(firing){
      if(t.get() < FIRING_TIME){
        launchmotor1.set(LAUNCH_SPEED);
        feedmotor.set(FEED_SPEED);
        return;
      } else {
        updatestate();
        t.reset();
        launchmotor1.set(0);
        firing = false;
      }
    }

    if(state == 0){ //no balls in system, waits here until balls enter
      feedmotor.set(0);
      updatestate();
      if(state == 0){
        return;
      }
    }

    if(state == 10){ //one ball in lowest position, sends it up to the second position
      feedmotor.set(FEED_SPEED);
      updateballs();
      if(mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state == 21){ //two balls, middle and bottom, sends them up a notch
      feedmotor.set(FEED_SPEED);
      updateballs();
      if(top && mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state % 10 == 2 || state == 23 || state == 11){ // A BALL IS AT THE TOP!!! READY TO LAUNCH!!
      feedmotor.set(0);
      if(controller.getRightTriggerAxis() > .9){
        firing = true;
        t.start();
      }
      updatestate();
    }

    
    // This method will be called once per scheduler run
    // if(controller.getAButtonPressed()){
    //   runmotor = !runmotor;
    // }
    // if(runmotor){
    //   motor1.set(controller.getRightTriggerAxis() - controller.getLeftTriggerAxis());
    // } else {
    //   motor1.set(0.0);
    // }
    
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
