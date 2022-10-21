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
  private final CANSparkMax motor1 = new CANSparkMax(17, MotorType.kBrushless);
  private final CANSparkMax motor2 = new CANSparkMax(1, MotorType.kBrushless);
  private final AnalogPotentiometer distanceSensor = new AnalogPotentiometer(0); 
  private final XboxController controller = new XboxController(0);
  private final Timer t = new Timer();
  boolean runmotor = false;
  
  public ExampleSubsystem() {
    motor1.restoreFactoryDefaults();
    motor2.restoreFactoryDefaults();
    motor1.setIdleMode(IdleMode.kCoast);
    motor2.setIdleMode(IdleMode.kCoast);
    motor2.follow(motor1);
    
  }

  public double getSpeed(double x){

    return(1 - x/.65);
  }

  @Override
  public void periodic() {
    System.out.println(distanceSensor.get());
    // if(controller.getAButtonPressed()){
    //   t.reset();
    //   t.start();
    //   runmotor = true;
    // }
    
    motor1.set(getSpeed(distanceSensor.get()));
    
    
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
