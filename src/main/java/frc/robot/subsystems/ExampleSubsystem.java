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
  private final double BALL_DETECTED3 = .123;
  private final double FEED_SPEED = -.6;
  private final double LAUNCH_SPEED = -.25;
  private final double FIRING_TIME = 1;
  boolean bot = false;
  boolean mid = false;
  boolean top = false;
  boolean firing = false;
  boolean idle = true;
  boolean constantfire = false;
  int state = 0; //keeps track of the balls in the system
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

  public int booltoint(boolean a){ //give 1 if true
    return a ? 1 : 0;
  }

  public void updateballs(){ //checks if a ball is in each spot
    top = irtop.get() > BALL_DETECTED3;
    mid = irmid.get() > BALL_DETECTED2;
    bot = irbot.get() > BALL_DETECTED;
  }

  public void updatestate(){ //uses updateballs to convert position of balls into a state number
    //the state variable keeps track of which state the mechanism is in
    //updatestate will not always be called when balls are being pushed by the conveyor, since it will be inaccurate

    //possible states are 0,  10 11 12,  21 22 23,  32
    //first digit is the number of balls in the system
    //second digit is the highest position of the balls
    //EXCEPTION: the case of ball - no ball - ball is state 23

    //List of states & which states they flow to:

    // State 0 (0 balls): Idling, constantly checking for new balls => State 10 (new ball)
    // State 10 (1 ball, bottom position): Moves ball up into middle position => State 11 (finished moving)
    // State 11 (1 ball, middle position): Idling, checking for new balls, waiting to fire => State 0 (fired), State 21 (new ball)
    // State 12 (1 ball, top position): Idling, checking for new balls, waiting to fire => State 0 (fired), State 23 (new ball)
    // State 21 (2 balls, bottom & middle positions): Moves balls up into middle and top positions => State 22 (finished moving)
    // State 22 (2 balls, middle & top positions): Idling, checking for new balls, waiting to fire => State 12 (fired), State 32 (new ball)
    // State 23 (2 balls, bottom & top positions): Idling, waiting to fire => State 11 (fired)
    // State 32 (3 balls, bottom, middle, & top positions): Idling, waiting to fire => State 22 (fired)

    updateballs();
    int t = booltoint(top);
    int m = booltoint(mid);
    int b = booltoint(bot);
    state = 0;
    state += 10 * (t + m + b);
    if(top){ 
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
  public void periodic() { //Implements logic as described above according to the current state
    //additionally has the ability to fire every ball currently stored

    //A button is used to activate and deactivate
    if(controller.getAButtonPressed()){
      active = !active;
    }

    //X button is used to fire entire storage
    if(controller.getXButtonPressed()){
      constantfire = true;
    }

    System.out.println(state + " " + irbot.get() + " " + irmid.get() + " " + irtop.get());


    if(!active){
      feedmotor.set(0);
      launchmotor1.set(0);
      return; 
    }

    if(constantfire){ //When constant fire is active, all balls will be fired until none are detected (State 0)
      updatestate();
      if(state == 0){
        feedmotor.set(0);
        launchmotor1.set(0);
        constantfire = false;
        firing = false;
        return;
      }
      feedmotor.set(FEED_SPEED);
      launchmotor1.set(LAUNCH_SPEED);
      return;
    }

    if(firing){ //When single fire is triggered, one ball is shot out 
      if(t.get() < FIRING_TIME){
        launchmotor1.set(LAUNCH_SPEED);
        feedmotor.set(FEED_SPEED);
        return;
      }

      //once firing is done, reset and figure out new state
      updatestate();
      t.stop();
      t.reset();
      launchmotor1.set(0);
      firing = false;
      return;
    }

    if(state == 0){ //State 0 (0 balls): Idling, constantly checking for new balls => State 10 (new ball)
      feedmotor.set(0);
      launchmotor1.set(0);
      updatestate();
      if(state == 0){
        return;
      }
    }

    if(state == 10){ //State 10 (1 ball, bottom position): Moves ball up into middle position => State 11 (finished moving)
      feedmotor.set(FEED_SPEED);
      updateballs();
      if(mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state == 21){ //State 21 (2 balls, bottom & middle positions): Moves balls up into middle and top positions => State 22 (finished moving)
      feedmotor.set(FEED_SPEED);
      updateballs();
      if(top && mid){
        updatestate();
      } else {
        return;
      } 
    }

    if(state % 10 == 2 || state == 23 || state == 11){ // States 11, 12, 22, 23, 32 (ball ready to fire): Waits for firing command while updating state
      feedmotor.set(0);
      launchmotor1.set(0);
      if(controller.getRightTriggerAxis() > .9){
        firing = true;
        t.start();
      }
      updatestate();
    }
    
    
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
