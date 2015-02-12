
//Comments are fun!
package org.usfirst.frc.team292.robot;

import edu.wpi.first.wpilibj.CANJaguar;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {
	Joystick controller;
	Joystick operator;
	Talon lf,rf,lr,rr,arm;
	CANJaguar liftMotor,gripMotor;
	RobotDrive drive;
	Camera cam;
	Gyro gyro;
	Encoder liftEncoder;
	Encoder driveEncoder;
	Counter counter;
	Servo armHolder;
	int autonomusMode=2;
	
	Dashboard dashboard;
	boolean change = false;
	
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	controller = new Joystick (0);
    	operator = new Joystick(1);
    	lf = new Talon (0);
    	rf = new Talon (2);
    	lr = new Talon (1);
    	rr = new Talon (3);
    	arm = new Talon (6);
    	liftMotor = new CANJaguar(2);
    	gripMotor = new CANJaguar(3);
    	armHolder = new Servo(9);

    	drive = new RobotDrive (lf, lr, rf, rr);
    	drive.setInvertedMotor(MotorType.kFrontLeft, false);
    	drive.setInvertedMotor(MotorType.kFrontRight, true);
    	drive.setInvertedMotor(MotorType.kRearLeft, false);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    	
    	//cam = new Camera();
        //cam.setPriority(4);
    	//cam.start();
    	
    	gyro = new Gyro(0);
    	
    	liftEncoder = new Encoder(0,1);
    	driveEncoder = new Encoder(19,20);
    	driveEncoder.setDistancePerPulse(-0.017444444);
    	
    	counter = new Counter(8);
    	
    	dashboard = new Dashboard();
    	dashboard.setGyro(gyro);
    	dashboard.setLiftMotor(liftMotor);
    	dashboard.setLiftEncoder(liftEncoder);
    	dashboard.start();
    	
    	System.out.println("Robot Initialization Complete: " + Timer.getFPGATimestamp());
    }

    /**
     * This function is called before entering disabled
     */
    public void disabledInit() {
    	System.out.println("Entering Disabled: " + Timer.getFPGATimestamp());
    }

    /**
     * This function is called periodically during disabled
     */
    public void disabledPeriodic() {
    	//System.out.println("Counts: " + counter.get());
    	//System.out.println("Drive Encoder counts: " + driveEncoder.get());
    	//System.out.println("Lift  Encoder counts: " + liftEncoder.get());
    	//System.out.println("Jag bus voltage: " + liftMotor.getBusVoltage());
    	//System.out.println("Gyro angle: " + gyro.getAngle());
    }

    /**
     * This function is called before entering autonomous
     */
    double starttime, autoStateTime = 0;
    int autostate;
    
    public void autonomousInit() {
    	System.out.println("Entering Autonomous: " +(starttime = Timer.getFPGATimestamp()));
    	driveEncoder.reset();
		gyro.reset();
		counter.reset();
		autostate=0;
		autoStateTime = starttime;
    }

    /**
     * This function is called periodically during autonomous
     */
    
    
    public void autonomousPeriodic() {
    	System.out.println("Distance: " + driveEncoder.getDistance());
    	System.out.println("Angle:    " + gyro.getAngle());
    //	if(driveEncoder.getDistance() < 6.0)
    //	{
    //		drive.mecanumDrive_Cartesian(0, -0.2, -gyro.getAngle()/12, gyro.getAngle());
    //	} else {
    //		drive.stopMotor();
    //	}
    	switch (autonomusMode) {
    	
    	case 1: // wait then move to score zone
    		switch (autostate) {
    		case 0:    		
    			System.out.println("Time is: " + (Timer.getFPGATimestamp()-starttime));
	    		if (Timer.getFPGATimestamp()-starttime >= 5)
	    			autostate++;
    		break;
    		
    		case 1:    		
    			if (Timer.getFPGATimestamp()-starttime >= 10)
    				autostate++;
    				
    			drive.mecanumDrive_Cartesian(0, -.292,0, gyro.getAngle());
    		break;
    		
    		case 2:
    		default:
     			drive.stopMotor();
     		break;
    		}
    		
    	case 2: // pickup something and move to score zone
    		switch (autostate) {
    		case 0: // close the arms   		
    			gripMotor.set(-1);

	    		if(counter.get() > 70) {
	    			gripMotor.set(0);
	    			autoStateTime = Timer.getFPGATimestamp();
	    			autostate++;
	    		}
	    		else if (Timer.getFPGATimestamp()-starttime >= 10)
	    			autostate = 2; // timeout
    		break;
    		
    		case 1: // lift the something   		
    			liftMotor.set(-1.0);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 3) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 10)
    				autostate++;
    				
    		break;
    		
    		case 2: // drive to score zone
    			drive.mecanumDrive_Cartesian(-.3, 0,0, gyro.getAngle());
    			if (Timer.getFPGATimestamp() - autoStateTime > 3) 
    				autostate++;
    			if (Timer.getFPGATimestamp() - starttime > 14) 
    				autostate++;
    		break;
    			
    		case 3: // be done
    		default:
     			drive.stopMotor();
     		break;
    		}
    		
    		
    		
    	}
    	
    	
    	
    	
    	
    }

    /**
     * This function is called before entering operator control
     */
	int gripPosition; // initialize to unknown
	boolean gripDirection; // true for close direction
	public void teleopInit() {
    	System.out.println("Entering Operator Control: " + Timer.getFPGATimestamp());
    	int gripPosition=-1; // initialize to unknown
    }

    /**
     * This function is called periodically during operator control
     */
	boolean oldFwdLim = false;
	boolean oldRevLim = false;
	int oldLiftEncoder = 0; 
	public void teleopPeriodic() {
    	
    	double xSpeed = controller.getX();
    	if(Math.abs(xSpeed) < 0.05) xSpeed = 0.0;
    	
    	double ySpeed = controller.getY();
    	if(Math.abs(ySpeed) < 0.05) ySpeed = 0.0;
    	
    	double twistSpeed = controller.getZ();
    	if(twistSpeed < -0.175) {
    		twistSpeed += 0.175;
    	} else if (twistSpeed > 0.175) {
        	twistSpeed -= 0.175;
    	} else {
    		twistSpeed = 0.0;
    	}
		twistSpeed *= 0.45;
    	//comment = 0.25; This is too slow
    	double angle = 0;
    	if(controller.getRawButton(7)) gyro.reset();
    	if(controller.getTrigger()) {
    		angle = 0;
    	} else {
    		angle = gyro.getAngle();
    	}
    	drive.mecanumDrive_Cartesian(xSpeed, ySpeed, twistSpeed, angle);
    	
    	if(Math.abs(controller.getX()) > .1 || Math.abs(controller.getY()) >  .1 || Math.abs(controller.getTwist()) >  .1){
    		//cam.Flashy(true);
    	} else {
    		//cam.Flashy(false);
    	}
    	
    	//Now I will write you a lovely story.
    	//One time there was a girl named Taylor who was bored.
    	//She wasn't doing anything, so she typed in the code to pretend like she was doing stuff,
    	//So she wouldn't have to mess with the buttons.
    	//And it worked.
    	//And then one day Chandler saw her lovely stories.
    	//And he was like, "Taylor, what the heck did you do to the code?"
    	//And she laughed merrily.
    	//The end!
    	//applauds
    	//starts crying because it was so beautiful

    	double opY = operator.getY();
    	if(Math.abs(opY)>.5)
    		liftMotor.set(-opY);
    	else
    		liftMotor.set(0);
    	double opX = operator.getX();
    	if(Math.abs(opX)>.5) {
    		gripPosition=gripPosition-(int)Math.signum(opX)*counter.get();
    		counter.reset();
    		if(opX>0) 	// open
    			if(gripMotor.getForwardLimitOK())
    				gripMotor.set(opX);
    			else {
    				gripMotor.set(.2); // hold the arm open with a small force
    				counter.reset();
    				gripPosition=0;
    			}	
    		else		// close
    			if(gripPosition<80)
    				gripMotor.set(opX);
    			else
    				gripMotor.set(0);
    	}
    	else
    		gripMotor.set(0);

    	if(operator.getRawButton(10))
    		armHolder.set(0);	// release the arm
    	if(operator.getRawButton(11))
    		armHolder.set(.5);
    	
    	
    	if(operator.getRawButton(3))
    		//if(liftEncoder> 567) {
    		arm.set(.5);
    	//	}
    	else if(operator.getRawButton(2))
    		arm.set(-.7);
    	else
    		arm.set(0);
    		
    	if(oldLiftEncoder != liftEncoder.get()) {
    		System.out.println("Encoder counts: " + liftEncoder.get());
    		oldLiftEncoder = liftEncoder.get();
    	}
    	if(oldFwdLim != liftMotor.getForwardLimitOK()) {
    		System.out.println("JaguarForwardLimit:" + liftMotor.getForwardLimitOK()); 
    		oldFwdLim = liftMotor.getForwardLimitOK();
    	}
    	if(oldRevLim != liftMotor.getReverseLimitOK()) {
    		System.out.println("JaguarBackwardLimit:" + liftMotor.getReverseLimitOK());
    		oldRevLim = liftMotor.getReverseLimitOK();
    	}
    	
    	//System.out.println("gripPosition = " + gripPosition +" counts: " + counter.get());

    	if (!liftMotor.getForwardLimitOK()) { 
    		liftEncoder.reset();
    		//Matt, it is being mean and not working!!!!!
    	}
    }
    {
    	
    }
    
    /**
     * This function is called before entering test mode
     */
    public void testInit() {
    	System.out.println("Entering Test: " + Timer.getFPGATimestamp());
    }
    
    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
    
    }
    
}
//All of these random comments were brought to you by ~ Taylor!