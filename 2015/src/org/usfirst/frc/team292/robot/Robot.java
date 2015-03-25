
//Comments are fun!
package org.usfirst.frc.team292.robot;

import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.RobotDrive;
import edu.wpi.first.wpilibj.RobotDrive.MotorType;
import edu.wpi.first.wpilibj.SerialPort;
import edu.wpi.first.wpilibj.Talon;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.Servo;
import com.kauailabs.navx_mxp.AHRS;

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
	CANTalon liftMotor,gripMotor;
	RobotDrive drive;
	Camera cam;
	Gyro gyro;
	Encoder liftEncoder;
	Encoder driveEncoder;
	Counter counter;
	Servo armHolder;
	DigitalInput robotID;
	PowerDistributionPanel pdp;
	Lights lights;
	int autonomousMode;
	
	Dashboard dashboard;
	boolean change = false;
	boolean armHeld = true;

	SerialPort serial_port;
	AHRS navx; // This class can only be used w/the navX MXP.
	
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
    	liftMotor = new CANTalon(2);
    	gripMotor = new CANTalon(3);
    	armHolder = new Servo(9);
    	robotID = new DigitalInput(24);
    	pdp = new PowerDistributionPanel();
    	
    	lights = new Lights(2,0,3,1);

    	drive = new RobotDrive (lf, lr, rf, rr);
    	drive.setInvertedMotor(MotorType.kFrontLeft, false);
    	drive.setInvertedMotor(MotorType.kFrontRight, true);
    	drive.setInvertedMotor(MotorType.kRearLeft, false);
    	drive.setInvertedMotor(MotorType.kRearRight, true);
    	
    	CameraServer server;
    	server = CameraServer.getInstance();
    	server.setQuality(50);
    	if(isCompetitionBot()) {
    		cam = new Camera("cam0", "cam0");
    	} else {
    		cam = new Camera("cam1", "cam2");
    	}
        cam.setPriority(4);
    	cam.start();
    	
    	gyro = new Gyro(0);
    	
    	liftEncoder = new Encoder(0,1);
    	driveEncoder = new Encoder(19,20);
    	driveEncoder.setDistancePerPulse(-0.017444444);
    	
    	counter = new Counter(8);
    	
		try {
			serial_port = new SerialPort(57600, SerialPort.Port.kMXP);
			byte update_rate_hz = 50;
			navx = new AHRS(serial_port, update_rate_hz);
		} catch (Exception ex) {
			System.out.println("Error initializing NavX: " + ex.toString());
		}
    	
    	dashboard = new Dashboard();
    	dashboard.setGyro(gyro);
    	dashboard.setLiftMotor(liftMotor);
    	dashboard.setLiftEncoder(liftEncoder);
    	dashboard.setPDP(pdp);
    	dashboard.setNavX(navx);
    	dashboard.start();
    	
		while (navx.isCalibrating()) {
			// "wait until calibration is done" ~Matt
			// "Okay, and then after that. Hey stop that."
		}
		navx.zeroYaw();

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
		navx.zeroYaw();
		counter.reset();
		autostate=0;
		autoStateTime = starttime;
		autonomousMode = dashboard.getAutoMode();
		System.out.println("Autonomous Mode: " + autonomousMode);
    }

    /**
     * This function is called periodically during autonomous
     */
    
    
	public void autonomousPeriodic() {
    	//System.out.println("Distance: " + driveEncoder.getDistance());
    	//System.out.println("Angle:    " + getAngle());
    //	if(driveEncoder.getDistance() < 6.0)
    //	{
    //		drive.mecanumDrive_Cartesian(0, -0.2, -getAngle()/12, getAngle());
    //	} else {
    //		drive.stopMotor();
    //	}
    	switch (autonomousMode) {
    	case 0: // do nothing
    		drive.stopMotor();
    		break;
    	
    	case 1: // wait then move to score zone
    		switch (autostate) {
    		case 0:    		
    			System.out.println("Time is: " + (Timer.getFPGATimestamp()-starttime));
	    		if (Timer.getFPGATimestamp()-starttime >= 0.5) {
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
	    		}
    		break;
    		
    		case 1:    		
    			if (Timer.getFPGATimestamp()-autoStateTime >= 2) {
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    				
    			drive.mecanumDrive_Cartesian(0, 0.4, 0, 0);
    		break;
    		
    		case 2:
    		default:
     			drive.stopMotor();
     		break;
    		}
    		break;
    	case 2: // pickup recycle bin and move to score zone
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
    		
    		case 1: // lift the recycle bin   		
    			liftMotor.set(-1.0);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 3) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 10)
    				autostate++;
    				
    		break;
    		
    		case 2: // drive to score zone BACKWARDS
    			drive.mecanumDrive_Cartesian(-.6, 0,-getAngle()/12, getAngle());
    			if (Timer.getFPGATimestamp() - autoStateTime > 4.0) 
    				autostate++;
    			if (Timer.getFPGATimestamp() - starttime > 14) 
    				autostate++;
    		break;
    			
    		case 3: // be done
    		default:
     			drive.stopMotor();
     		break;
     		
     		
    		}
    		break;
    	case 3: // pickup tote and move to score zone
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
    		
    		case 1: // lift the tote 		
    			liftMotor.set(-1.0);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 2) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 10)
    				autostate++;
    				
    		break;
    		
    		case 2: // drive to score zone
    			drive.mecanumDrive_Cartesian(.6, 0,-getAngle()/12, getAngle());
    			if (Timer.getFPGATimestamp() - autoStateTime > 4.0) 
    				autostate++;
    			if (Timer.getFPGATimestamp() - starttime > 14) 
    				autostate++;
    		break;
    			
    		case 3: // be done
    		default:
     			drive.stopMotor();
     		break;
     		
    		}
    		break;
    	case 4: // pickup recycle bin and place on tote and move to score zone
    		switch (autostate) {
    		case 0: // close the arms   		
    			gripMotor.set(-1);

	    		if(counter.get() > 50) {
	    			gripMotor.set(0);
	    			autoStateTime = Timer.getFPGATimestamp();
	    			autostate++;
	    		}
	    		else if (Timer.getFPGATimestamp()-starttime >= 10)
	    			autostate = 2; // timeout
    		break;
    		
    		case 1: // lift the recycle bin   		
    			liftMotor.set(-1.0);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 3) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 10)
    				autostate++;
    				
    		break;
    		
    		case 2: //drive to the tote
    			drive.mecanumDrive_Cartesian(0, -.3 ,0, getAngle());
    			if (Timer.getFPGATimestamp()-autoStateTime > 1) {
    				drive.stopMotor();
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 10)
        				autostate++;
    			
    		break;
    		
    		case 3: // drop recycle bin  lower arms
    			
    			liftMotor.set(1);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 1) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			
    			if (Timer.getFPGATimestamp()-starttime >= 10)
        				autostate++;
    			break;
    			
    		case 4: // Open the arms to drop recycle bin
    			gripMotor.set(1);

	    		if(!gripMotor.isFwdLimitSwitchClosed() || Timer.getFPGATimestamp()-autoStateTime > 1) {
	    			gripMotor.set(0);
	    			counter.reset();
	    			autoStateTime = Timer.getFPGATimestamp();
	    			autostate++;
	    		}
	    		else if (Timer.getFPGATimestamp()-starttime >= 13)
	    			autostate ++; // timeout
    		break;
    		
    		case 5: // lower arms to pickup tote and bin
    			
    			liftMotor.set(1);
    			
    			if (!liftMotor.isFwdLimitSwitchClosed()) {  // I don't know if this should be the reverse limit or the forward limit
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			
    			if (Timer.getFPGATimestamp()-starttime >= 13)
        				autostate++;
    			break;

        		
    		case 6: //drive to the tote
    			drive.mecanumDrive_Cartesian(0, -.3 ,0, getAngle());
    			if (Timer.getFPGATimestamp()-autoStateTime > .4) {
    				drive.stopMotor();
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 13)
        				autostate++;
    			
    		break;
    		
    		case 7: // close arms on tote for pickup
    			gripMotor.set(-1);

	    		if(counter.get() > 70) {
	    			gripMotor.set(0);
	    			autoStateTime = Timer.getFPGATimestamp();
	    			autostate++;
	    		}
	    		else if (Timer.getFPGATimestamp()-starttime >= 13)
	    			autostate ++; // timeout
    		break;
    		
    		case 8: // lift the tote bin combo   		
    			liftMotor.set(-1.0);
    			
    			if (Timer.getFPGATimestamp()-autoStateTime > 1) {
    				liftMotor.set(0);
    				autoStateTime = Timer.getFPGATimestamp();
    				autostate++;
    			}
    			if (Timer.getFPGATimestamp()-starttime >= 13)
    				autostate++;
    				
    		break;
    		
    		case 9: // drive to score zone
    			drive.mecanumDrive_Cartesian(-.6, 0,-getAngle()/12, getAngle());
    			if (Timer.getFPGATimestamp() - autoStateTime > 6) 
    				autostate++;
    			if (Timer.getFPGATimestamp() - starttime > 15) 
    				autostate++;
    		break;
    			
    			
    		default:
     			drive.stopMotor();
     		break;
    		}
    	}
    	
    	lights.periodic();
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
    	xSpeed *= xSpeed * Math.signum(xSpeed);
    	
    	double ySpeed = controller.getY();
    	if(Math.abs(ySpeed) < 0.05) ySpeed = 0.0;
    	ySpeed *= ySpeed * Math.signum(ySpeed);
    	
    	double twistSpeed = controller.getZ();
    	if(twistSpeed < -0.175) {
    		twistSpeed += 0.175;
    	} else if (twistSpeed > 0.175) {
        	twistSpeed -= 0.175;
    	} else {
    		twistSpeed = 0.0;
    	}
    	twistSpeed *= twistSpeed * Math.signum(twistSpeed) * 0.8;
		//twistSpeed *= 0.45;
    	//comment = 0.25; This is too slow
    	if(controller.getRawButton(7)) navx.zeroYaw();
    	
    	double angle = 0;
    	switch(dashboard.getDriveMode()) {
    	case 0:
    		angle = controller.getTrigger() ? getAngle() : 0;
    		break;
    	case 1:
    		angle = controller.getTrigger() ? 0 : getAngle();
    		break;
		default:
			angle = 0;
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
		if (Math.abs(opY) > .05)
			liftMotor.set(-opY);
		else
			liftMotor.set(0);
		
		double opX = operator.getX();
		if (Math.abs(opX) > .05) {
			gripPosition = gripPosition - (int) Math.signum(opX) * counter.get();
			counter.reset();
			if (opX > 0) // open
				if (gripMotor.isFwdLimitSwitchClosed())
					gripMotor.set(opX);
				else {
					gripMotor.set(.2); // hold the arm open with a small force
					counter.reset();
					gripPosition = 0;
				}
			else // close
			if (gripPosition < 80)
				gripMotor.set(opX);
			else
				gripMotor.set(0);
		} else
			gripMotor.set(0);

    	if(operator.getRawButton(4)) armHeld = false;
    	else if(operator.getRawButton(5)) armHeld = true;
		
    	if(armHeld) armHolder.set(0); // hold the arm
    	else armHolder.set(0.5); // release the arm

    	if(operator.getRawButton(7)) cam.viewBottomCamera();
    	else if (operator.getRawButton(6)) cam.viewTopCamera();
    	
    	if(operator.getRawButton(3) && !armHeld) {// arm out - only if not being held
    		if(isCompetitionBot()) {
    			arm.set(0.7);
    		} else {
        		arm.set(-0.7);
    		}
    	}
    	else if(operator.getRawButton(2)) {// arm in
    		if(isCompetitionBot()) {
    			arm.set(-0.8);
    		} else {
        		arm.set(0.8);
    		}
    	}
    	else arm.set(0);
    	
    		
    	if(oldLiftEncoder != liftEncoder.get()) {
    		System.out.println("Encoder counts: " + liftEncoder.get());
    		oldLiftEncoder = liftEncoder.get();
    	}
    	if(oldFwdLim != liftMotor.isFwdLimitSwitchClosed()) {
    		System.out.println("JaguarForwardLimit:" + liftMotor.isFwdLimitSwitchClosed()); 
    		oldFwdLim = liftMotor.isFwdLimitSwitchClosed();
    	}
    	if(oldRevLim != liftMotor.isRevLimitSwitchClosed()) {
    		System.out.println("JaguarBackwardLimit:" + liftMotor.isRevLimitSwitchClosed());
    		oldRevLim = liftMotor.isRevLimitSwitchClosed();
    	}
    	
    	//System.out.println("gripPosition = " + gripPosition +" counts: " + counter.get());

    	if (!liftMotor.isFwdLimitSwitchClosed()) { 
    		liftEncoder.reset();
    	}
    	
    	if(operator.getRawButton(10)) lights.setCoopertition(false);
    	if(operator.getRawButton(11)) lights.setCoopertition(true);
    	lights.periodic();
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
    
    /**
     * This function reads the digital input corresponding to the
     * jumper only located on the competition bot
     */
    public boolean isCompetitionBot() {
    	return !robotID.get();
    }
    
    /**
     * This function returns the current angle of the robot as determined by the NavX
     */
    public double getAngle() {
    	if(navx.isConnected()) {
    		return navx.getYaw();
    	} else {
    		return 0.0;
    	}
    }
}
//All of these random comments were brought to you by ~ Taylor!

















//                                                     I BROKE THE CODE!!!












