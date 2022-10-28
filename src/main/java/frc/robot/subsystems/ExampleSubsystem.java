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


public class ExampleSubsystem extends SubsystemBase {
  /** Creates a new ExampleSubsystem. */
  private final CANSparkMax launchmotor1 = new CANSparkMax(3, MotorType.kBrushless);
  private final CANSparkMax launchmotor2 = new CANSparkMax(16, MotorType.kBrushless);//check if these should be opposite of each other
  private final CANSparkMax feedmotor = new CANSparkMax(8, MotorType.kBrushless);
  private final AnalogPotentiometer irbot = new AnalogPotentiometer(0);//at the bottom
  private final AnalogPotentiometer irmid = new AnalogPotentiometer(1);//at the middle
  private final AnalogPotentiometer irtop = new AnalogPotentiometer(3);//at the top
  private final XboxController controller = new XboxController(0);
  private final Timer t = new Timer();
  private final double BALL_DETECTED = .01;//SET THIS THING!!!!
  private final double FEED_SPEED = .5;
  private final double LAUNCH_SPEED = .1;
  private final double FIRING_TIME = .5;
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
    feedmotor.restoreFactoryDefaults();
    launchmotor1.setIdleMode(IdleMode.kCoast);
    launchmotor2.setIdleMode(IdleMode.kCoast);
    feedmotor.setIdleMode(IdleMode.kCoast);
    launchmotor2.follow(launchmotor1);
    
  }

  // public double getSpeed(double x){

  //   return(1 - x/.65);
  // }
  public int booltoint(boolean a){
    return a ? 1 : 0;
  }

  public void updateballs(){
    top = irtop.get() < BALL_DETECTED;
    mid = irmid.get() < BALL_DETECTED;
    bot = irbot.get() < BALL_DETECTED;
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
      if(!mid){
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
    // System.out.print("1: " + ir1.get());
    // System.out.print("  2: " + ir1.get());
    // System.out.println("  3: " + ir1.get());
    if(controller.getAButtonPressed()){
      active = !active;
    }

    System.out.println(state);
    if(!active){
      return; 
    }

    if(firing){
      if(t.get() < FIRING_TIME){
        launchmotor1.set(LAUNCH_SPEED);
        return;
      } else {
        updatestate();
        t.reset();
        launchmotor1.set(LAUNCH_SPEED);
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

    if(state == 10 || state == 23){ //one ball in lowest position, sends it up to the second position
      feedmotor.set(FEED_SPEED);
      if(mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state == 11){ //one ball in middle position, sends it to the top
      feedmotor.set(FEED_SPEED);
      if(top){
        updatestate();
      } else {
        return;
      } 
    }

    if(state == 21){ //two balls, middle and bottom, sends them up a notch
      feedmotor.set(FEED_SPEED);
      if(top && mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state % 10 == 2){ // A BALL IS AT THE TOP!!! READY TO LAUNCH!!
      feedmotor.set(0);
      if(controller.getRightTriggerAxis() > .9){
        firing = true;
        t.start();
      }
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
