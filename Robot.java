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

package frc.robot;
// the com.revrobotics is weird. i dont think it's even needed
// deleted the rev thing 3/28/22
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase; 
import edu.wpi.first.wpilibj.TimedRobot; 
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.buttons.Button;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Compressor;
import com.ctre.phoenix.motorcontrol.*;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.ctre.phoenix.motorcontrol.can.TalonSRXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.Timer;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String m_autoSelected;
  private final SendableChooser<String> m_chooser = new SendableChooser<>();
// defining the motors (we used falcon 500) they use talon fx//
  private TalonFX m_Left0 = new TalonFX(1);
  private TalonFX m_Left1 = new TalonFX(2);
  private TalonFX m_Right0 = new TalonFX(3);
  private TalonFX m_Right1 = new TalonFX(4);
  //if this doesnt work then i guess il have to add more code for the motors instead of grouping them
  //why does this not work? it should... right?
  //checked the website this was the thing they gave... code... why must you do this to me... 3/28/22
  //it works despite the errors 3/29/22
  private MotorControllerGroup m_left = new MotorControllerGroup(m_Left0, m_Left1);
  private MotorControllerGroup m_right = new MotorControllerGroup(m_Right0, m_Right1);
  private DifferentialDrive m_Drive = new DifferentialDrive(m_left,m_right);
  private final Timer m_timer = new Timer();
// everything else used other stuff, so talon srx = sparkmax i guess//
  WPI_TalonSRX winchMotor;
  WPI_TalonSRX intakeMotor1;
  WPI_TalonSRX intakeMotor2;
  WPI_TalonSRX shooterMotor;
// xbox superiority, suck it sony!
  private XboxController xbox1 = new XboxController(0);
  private XboxController xbox2 = new XboxController(0);
//idk what this does
  private boolean m_LimelightHasValidTarget = false;
  private double m_LimelightDriveCommand = 0.0;
  private double m_LimelightSteerCommand = 0.0;

 
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
          winchMotor = new WPI_TalonSRX(5);
          intakeMotor1 = new WPI_TalonSRX(6);
          intakeMotor2 = new WPI_TalonSRX(7);
          shooterMotor = new WPI_TalonSRX(8);
           // oh yeah gotta invert the right motors because WPI is tired of our lazyness and makes us have to add that bit in
    m_right.setInverted(true);

  
    
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
        m_Drive.stopMotor();}
  }
  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {}

  /** This function is called periodically during operator control. */
  // FIX SOMETHING ABOUT HAND!!!!! FIND A WAY TO CONNECT XBOX TO ROBOT!!!! 3/19/22
  // 3/28/22 i replaced the hand thing with the getrightx and getleftx. hope it works. idk what it even does in the first place
  @Override
  public void teleopPeriodic() {

        Update_Limelight_Tracking();
      
        double steer = xbox1.getRightX(); xbox1.getLeftX();
        double drive = -xbox1.getRightY(); xbox1.getLeftY();
        boolean auto = xbox1.getRightStickButton();

        steer *= 0.70;
        drive *= 0.70;

        if (auto)
        {
          if (m_LimelightHasValidTarget)
          {
                m_Drive.arcadeDrive(m_LimelightDriveCommand,m_LimelightSteerCommand);
          }
          else
          {
                m_Drive.arcadeDrive(0.0,0.0);
          }
        }
        else
        {
          m_Drive.arcadeDrive(drive,steer);
        }
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {
    m_Drive.arcadeDrive(0,0);
    intakeMotor1.set(0);
    intakeMotor2.set(0);
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
  public void Update_Limelight_Tracking()
  {
        // These numbers must be tuned for your Robot!  Be careful!
        //to test for later 3/19/22 idk what this does, but it looks important. keep it
        //wtf is this limelight stuff?
        //this part is not in the given template originally, some some older member prolly made this thing. 
        final double STEER_K = 0.05;                    // how hard to turn toward the target
        final double DRIVE_K = 0.3;                    // how hard to drive fwd toward the target
        final double DESIRED_TARGET_AREA = 13.0;        // Area of the target when the robot reaches the wall
        final double MAX_DRIVE = 0.7;                   // Simple speed limit so we don't drive too fast

        double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);
        double tx = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tx").getDouble(0);
        double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
        double ta = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ta").getDouble(0);

        if (tv < 1.0)
        {
          m_LimelightHasValidTarget = false;
          m_LimelightDriveCommand = 0.0;
          m_LimelightSteerCommand = 0.0;
          return;
        }

        m_LimelightHasValidTarget = true;

        // Start with proportional steering
        double steer_cmd = tx * STEER_K;
        m_LimelightSteerCommand = steer_cmd;

        // try to drive forward until the target area reaches our desired area
        double drive_cmd = (DESIRED_TARGET_AREA - ta) * DRIVE_K;

        // don't let the robot drive too fast into the goal
        if (drive_cmd > MAX_DRIVE)
        {
          drive_cmd = MAX_DRIVE;
        }
        m_LimelightDriveCommand = drive_cmd;
  }



   
//for two controllers
public void twoControl(){
       //change values based on motor placement. postive numbers are clockwise, negatives are clock wise
         //intake for ball keys are defined 
     boolean in = xbox1.getYButton();
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

     while(in) {
       intakeMotor1.set(0.8);
       intakeMotor2.set(-0.8);
     }
    while(out) {
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
  while(aiml) {
    m_left.set(-.1);
    m_right.set(.1);
  }
  while(aimr) {
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
boolean down = xbox2.getBButton();
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
steer *= .7;
//to for driving. if someone can parallel park the damn thing that would be nice.
if(drive > 0.05) {
  while(drive > 0.05) {
    m_Drive.arcadeDrive(drive, steer);
  }

}

//climber
while(up) {
  winchMotor.set(0.4);
}
while(down) {
  winchMotor.set(-0.7);
}

}
     //for one controler
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
     double drive = xbox1.getLeftY();
     double steer = xbox1.getLeftX();
     //to lower speed
     drive *= .3;
     steer *= .3;
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
    if(drive > 0.05) {
      while(drive > 0.05) {
        m_Drive.arcadeDrive(drive, steer);
      }

    }

     //intake

     while(in) {
       intakeMotor1.set(0.8);
       intakeMotor2.set(-0.8);
     }
    while(out) {
      intakeMotor1.set(-0.8);
      intakeMotor2.set(0.8);
    }
//climber
    while(up) {
      winchMotor.set(0.4);
    }
    while(down) {
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


  


