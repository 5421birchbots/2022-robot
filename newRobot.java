// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
/** 
people that are doing this cannot code, please learn how to code and dont be like us - J.C + E.L 3/19/22
Also, please work on the robot instead of spending too much time on prototyping and design. just pick something and go. Function over efficency
If it works, it works! if it doesn't, ignore your mistakes.
dont fuck up 
ctrl c and ctrl v is your best friend
*/

// the original frankencode has issues with some compile classpath and the build gradle is blind. It can't see the main method despite it being there//
//copied the frankencode to here, and it works!. What in gods name happened over there?! oh well. 3/29/22

//removed some imports that are not needed 3/28/22
//the motorcontrollergroup works despite the "error" 3/28/22
//hopefully this thing works.
// 4/7/22 it can drive. its 830pm rn and there is so much to test and debug
// removed the limelight stuff. it is not needed
// put the code in teleopperiodic because thats where the magic happens

//4/18/22 use if else not while loops. climber reverses if you move up too much. driving could be slower, maybe have an external button to press to unlock faster speeds?
// kinda want to make code for other things like sensors of pneumatics for future situations.
package frc.robot;
// the com.revrobotics is weird. i dont think it's even needed
// deleted the rev thing 3/28/22
import edu.wpi.first.wpilibj.*;
//import edu.wpi.first.wpilibj.drive.*;
//import edu.wpi.first.wpilibj.Joystick;
//import edu.wpi.first.wpilibj.RobotBase; 
//import edu.wpi.first.wpilibj.TimedRobot; 
//import edu.wpi.first.wpilibj.command.Command;
//import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
//import edu.wpi.first.wpilibj.buttons.Button;
//import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
//import edu.wpi.first.wpilibj.drive.RobotDriveBase;
//import edu.wpi.first.networktables.NetworkTable;
//import edu.wpi.first.networktables.NetworkTableEntry;
//import edu.wpi.first.wpilibj.DoubleSolenoid;
//import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.*;
//import com.ctre.phoenix.motorcontrol.can.TalonSRX;
//import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
//import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
//import edu.wpi.first.wpilibj.DigitalInput;
//import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
//import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
//import edu.wpi.first.wpilibj.CAN;
//import edu.wpi.first.wpilibj.motorcontrol.Spark;
//import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class newRobot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
// defining the motors (we used falcon 500) they use talon fx//
//use wpi_talonfx not talonfx
  private WPI_TalonFX m_Left0 = new WPI_TalonFX(1);
  private WPI_TalonFX m_Left1 = new WPI_TalonFX(2);
  private WPI_TalonFX m_Right0 = new WPI_TalonFX(3);
  private WPI_TalonFX m_Right1 = new WPI_TalonFX(4);
  //if this doesnt work then i guess il have to add more code for the motors instead of grouping them
  //why does this not work? it should... right?
  //checked the website this was the thing they gave... code... why must you do this to me... 3/28/22
  // it works despite the errors 3/29/22
  //some of the motors had different ID's test this depending on installation of motors.
  //the motors are ordered like this and it works! huzzah!
  private MotorControllerGroup m_left = new MotorControllerGroup(m_Right0, m_Left1);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_Left0, m_Right1);
  private DifferentialDrive m_Drive = new DifferentialDrive(m_left, m_right);
  private final Timer m_timer = new Timer();
// everything else used other stuff, so talon srx = sparkmax i guess//
  CANSparkMax winchMotor;
  CANSparkMax intakeMotor1;
  CANSparkMax shooterMotor;
// xbox superiority, suck it sony!
  private XboxController xbox1 = new XboxController(0);
  private XboxController xbox2 = new XboxController(1);
//idk what this does
  
 
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    m_Left0.set(ControlMode.PercentOutput, 0);
    m_Left1.set(ControlMode.PercentOutput, 0);
    m_Right0.set(ControlMode.PercentOutput, 0);
    m_Right1.set(ControlMode.PercentOutput, 0);
    //idk what this is, so we'll keep it there 
    m_chooser.setDefaultOption("Default Auto", kDefaultAuto);
    m_chooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", m_chooser);
    //defining the motors for the motor controllers
          winchMotor = new CANSparkMax(5, MotorType.kBrushless);
          intakeMotor1 = new CANSparkMax(4, MotorType.kBrushless);
          shooterMotor = new CANSparkMax(6, MotorType.kBrushless);
           // oh yeah gotta invert the right motors because WPI is tired of our lazyness and makes us have to add that bit in
    m_right.setInverted(true);
   // m_left.setInverted(true);

  
    
// robot starts at rest and the motors for the other functions that isnt driving is defined
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {}

  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well
   */
  @Override
  public void autonomousInit() {
    m_autoSelected = m_chooser.getSelected();
    // m_autoSelected = SmartDashboard.getString("Auto Selector", kDefaultAuto);
    System.out.println("Auto selected: " + m_autoSelected);
    m_timer.reset();
    m_timer.start();
  }
// we'll just leave this there
  /** This function is called periodically during autonomous. */

  //gotta work on this one, if only we can get the robot running first... 3/28/22 

  //...Still waiting... 3/30/22
  
  @Override
    public void autonomousPeriodic() {
      // 3/19/22 find a way to get the robot autonomous for 15 seconds
      if (m_timer.get() < 15) {
        m_Drive.arcadeDrive(0.5, 0.0); // drive forwards half speed
      } else {
        m_Drive.stopMotor();
      }
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    if (m_Drive != null){
        m_Drive.stopMotor();
    }
    oneControl();
  }

  /** This function is called periodically during operator control. */
  // FIX SOMETHING ABOUT HAND!!!!! FIND A WAY TO CONNECT XBOX TO ROBOT!!!! 3/19/22
  // 3/28/22 i replaced the hand thing with the getrightx and getleftx. hope it works. idk what it even does in the first place
  @Override
  // THIS IS THE CLASS THAT RUNS TO THE CONTROLLLLLLLLLLSSS THIS IS THE ONE PUT THE CODE IN HERE!!! 4/7/22
  public void teleopPeriodic() {
        OneControl();
//xbox 1 drives and intakes
//xbox 2 climbs and shoots
        // TEST BASED ON MOTOR PLACEMENT
      // it uses the right joystick, now left
      // change as you see fit
        double steer = xbox1.getRightX(); 
        double drive = xbox1.getLeftY(); 
       
// adjust the speed as you see fit
//steer was set to .5 and drive to .7 for handling purposes
        steer *= 0.30; 
        drive *= 0.30;

    //code for driving
    // its janky as you need to stop in order to turn, but it works i guess
          m_Drive.arcadeDrive(drive,steer);
            //bind these keys as you see fit
     //intake for ball keys are defined 
     // right trigger pulls in, left trigger spits out
     double in = xbox1.getRightTriggerAxis();
     double out = xbox1.getLeftTriggerAxis();
     // keys are defined for climber
     boolean up = xbox2.getYButton();
     boolean down = xbox2.getAButton();
     //key for shooter
     // coding the triggers is a bit iffy, needs test 3/29/22
     double shoot = xbox2.getRightTriggerAxis();
     double load = xbox2.getLeftTriggerAxis();
//timer i'm having the shooter run for a duration instead of holding the button

     //starts at zero
     //key for driving (this is one controler operating only so it has limited drive)
     //left joystick drives it (move it up or down), moving it left or right will turn it ideally
     //4/7/22 controls dont work. its on right joystick
     //4/7/22 now configured for left
     
/**I dont know what kind of driving controls people want. do they want fps style controls where left joystick moves while right joystick turns?
 * or do they want a driving game type control where B accelerates and left joystick steers?
 * the possibilities are almost endless!
 */
//JONAH YOU CHANGED THIS REMEMBER 
    //System.out.println("Winch encoder: " + winchMotor.getEncoder().getPosition());
     // winchMotor.set(0);
     //shooterMotor.set(0);
     //intakeMotor1.set(0);
     //intakeMotor2.set(0);
     //driving slow not for competitions
     //needs test, might work. if it doesnt, gonna need a lot more code.
    

     //intake

     if (in > 0.05) {
       intakeMotor1.set(0.8);
       //intakeMotor2.set(-0.8);
     }
    if (out > 0.05) {
      intakeMotor1.set(-0.8);
      //intakeMotor2.set(0.8);
    }
//climber
    if (up) {
      winchMotor.set(0.4);
    }
    if (down) {
      winchMotor.set(-0.4);
    }
//shooter 
// one button preps the wheel, the other one pushes it towards the shooter
// shooter wheel will get ready for 2 seconds, after 1 second the intake wheel will push the ball to the shooter wheel..... hopefully
if  (shoot > .05) {
  shooterMotor.set(1);
}
//test based on motor placement
if (load > .05) {
  intakeMotor1.set(1);
}
  } 
  

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_Drive.arcadeDrive(0,0);
    intakeMotor1.set(0);
    shooterMotor.set(0);
    winchMotor.set(0);
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() {}

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}

//copy starts here//
  
  /**
   * This function implements a simple method of generating driving and steering commands
   * based on the tracking data from a limelight camera.
   */
 



   
//for two controllers
// the code in it is good, the class is useless. keep it and copy the code to teleop
public void OneControl(){
       //change values based on motor placement. postive numbers are clockwise, negatives are clock wise
         //intake for ball keys are defined 
     boolean in = xbox1.getXButton();
     boolean out = xbox1.getBButton();
     // finer aim
     boolean aiml =xbox1.getLeftStickButton();
     boolean aimr= xbox1.getRightStickButton();
     //key for shooter
     boolean shoot = xbox1.getLeftStickButtonPressed();
//timer i'm having the shooter run for a duration instead of holding the button
Timer time = new Timer();
     //starts at zero

     winchMotor.set(0);
     shooterMotor.set(0);
     intakeMotor1.set(0);
     intakeMotor2.set(0);

     //intake

     if (in) {
       intakeMotor1.set(0.8);
       intakeMotor2.set(-0.8);
     }
    if (out) {
      intakeMotor1.set(-0.8);
      intakeMotor2.set(0.8);
    }
//shooter 
// shooter wheel will get ready for 3 seconds, after 2 seconds the intake wheel will push the ball to the shooter wheel..... hopefully
if(shoot) {
  time.reset();
  time.start();
  //gets the shooter wheel/drum prepped and ready
  if(time.get() <= 2) {
      shooterMotor.set(-1);
    }
    //intake pushes the ball in to the drum, thus propelling it
  if(time.get() >= 1 && time.get() <= 2) {
    intakeMotor2.set(-0.8);

  }
  
  } else {
    //stops after use
    shooterMotor.set(0);
    intakeMotor2.set(0);
  }
  //for slight adjustment in aim. 
  //this code might be useless. who knows!
  if(aiml) {
    m_left.set(-.1);
    m_right.set(.1);
  }
  if(aimr) {
    m_left.set(.1);
    m_right.set(-.1);
  }
   // other controler stuff is defined
   // idk if i can make it so one side runs on one speed while the other runs on a different one. hope this one works 3/28/22
   //driving forwads triggers the intake, but the other controller also can control it. the other one can also spit out the balls if needed
   //boolean front = xbox2.getBbutton();
   //boolean back = xbox2.getAButton();
   //archiving this just in case
   
// keys are defined for climber
boolean up = xbox2.getYButton();
boolean down = xbox2.getAButton();
   //boolean steer_left = xbox2.getLeftStickButton();
   //boolean steer_right= xbox2.getRightStickButton();
   //this code is messy, but hopefully it might work
   // this is so it can steer. it can rotate better at rest, and if you steer during driving it can turn. same for reverse. Kinda want to see someone parallel park the robot.
  // if(front && steer_left) {
   // while(front && steer_left) {
 //     m_left.set(.4);
 ////     m_right.set(.7);
 //     intakeMotor1.set(-.8);
 //     intakeMotor2.set(-.4);
 //   }
 //  } 
 //  else if(front) {
 //    while(front) {
 //     m_Drive.arcadeDrive(0.7,0);
  //    intakeMotor1.set(-.8);
  //    intakeMotor2.set(-.4);
 //    } 
 //  } 
 //  //this thing gets last priority. its mostly for rotation, use it only when at rest. great for angling shooter maybe.
 //  else if(steer_left) {
  //  while(steer_left) {
  //    m_left.set(-.3);
  //    m_right.set(.3);
 //   }
  // }
  // if (front && steer_right) {
  //   while(front && steer_right) {
   //    m_left.set(.7);
   //    m_right.set(.4);
   //    intakeMotor1.set(-.8);
   //    intakeMotor2.set(-.4);
 //    }
 //  }else if(steer_right) {
 //    while(steer_right) {
 //      m_left.set(.3);
 //      m_right.set(-.3);
 //    }
 //  }
 //  if(back && steer_left) {
 // while(back && steer_left) {
  //  m_left.set(-.3);
  //  m_right.set(-.2);
 // } 
//}else if(back) {
 // while(back) {
   // m_Drive.arcadeDrive(-.4,0);
 // }
//}
//if(back && steer_left) {
 // m_right.set(-.2);
  //m_left.set(-.3);
//}
//just in case
//if(steer_left && steer_right) {
 // m_Drive.arcadeDrive(0,0);
//}
//left joystick drives it (move it up or down), moving it left or right will turn it ideally
double drive = xbox1.getLeftY();
double steer = xbox1.getLeftX();
//to lower speed
drive *= .7;
steer *= .4;
//to for driving. if someone can parallel park the damn thing that would be nice.
if (drive > 0.05) {
    m_Drive.arcadeDrive(drive, steer);
  }



//climber
if (up) {
  winchMotor.set(0.1);
}
if (down) {
  winchMotor.set(-0.3);
}

}
     //for one controler
     // doesnt work, but the code inside it does. put this in teleop and change as you see fit
  public void oneControl(){
      //bind these keys as you see fit
     //intake for ball keys are defined 
     boolean in = xbox1.getBButton();
     boolean out = xbox1.getYButton();
     // keys are defined for climber
     boolean up = xbox1.getRightBumper();
     boolean down = xbox1.getLeftBumper();
     //key for shooter
     // coding the triggers is a bit iffy, needs test 3/29/22
     double shoot = xbox1.getRightTriggerAxis();
//timer i'm having the shooter run for a duration instead of holding the button
Timer time = new Timer();
     //starts at zero
     //key for driving (this is one controler operating only so it has limited drive)
     //left joystick drives it (move it up or down), moving it left or right will turn it ideally
     //4/7/22 controls dont work. its on right joystick
     double drive = xbox1.getLeftY(); 

     double steer = xbox1.getRightX(); 


     //to lower speed
     drive *= .01;
     steer *= .01;

     if(drive > 0.05) {
        m_Drive.arcadeDrive(drive, steer);
    }
/**I dont know what kind of driving controls people want. do they want fps style controls where left joystick moves while right joystick turns?
 * or do they want a driving game type control where B accelerates and left joystick steers?
 * the possibilities are almost endless!
 */


     winchMotor.set(0);
     shooterMotor.set(0);
     intakeMotor1.set(0);
     intakeMotor2.set(0);
     //driving slow not for competitions
     //needs test, might work. if it doesnt, gonna need a lot more code.
    

     //intake

     if(in) {
       intakeMotor1.set(0.8);
       intakeMotor2.set(-0.8);
     }
    if(out) {
      intakeMotor1.set(-0.8);
      intakeMotor2.set(0.8);
    }
//climber
    if(up) {
      winchMotor.set(0.4);
    }
    if(down) {
      winchMotor.set(-0.4);
    }
//shooter 
// shooter wheel will get ready for 2 seconds, after 1 second the intake wheel will push the ball to the shooter wheel..... hopefully
if(shoot > .05) {
  time.reset();
  time.start();
  if(time.get() < 2) {
      shooterMotor.set(-1);
    }
  if(time.get() == 1) {
    intakeMotor2.set(-0.8);
    if(time.get() == 2) {
      intakeMotor2.set(0);
    }


  }
  
  } 
  }
}


  


