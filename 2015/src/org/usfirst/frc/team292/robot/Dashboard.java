package org.usfirst.frc.team292.robot;

import edu.wpi.first.wpilibj.CANJaguar;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {

    private boolean enabled;
    private Thread task;
    private boolean run = true;
    private Gyro gyro;
    private CANJaguar lift;
    private Encoder liftEncoder;
    private PowerDistributionPanel pdp;

    private class DashboardThread extends Thread {

        Dashboard db;

        DashboardThread(Dashboard db) {
            this.db = db;
        }

        public void run() {
            while (run) {
                if (db.enabled()) {
                	if(gyro != null) SmartDashboard.putNumber("Gyro", gyro.getAngle());
                	if(liftEncoder != null) SmartDashboard.putNumber("Lift", liftEncoder.get());
                	if(lift != null) SmartDashboard.putBoolean("Lift At Bottom", !lift.getForwardLimitOK());
                	if(lift != null) SmartDashboard.putBoolean("Lift At Top", !lift.getReverseLimitOK());
                	if(pdp != null) pdp.updateTable();
                	if(pdp != null) SmartDashboard.putData("PDP", pdp);
                }
                try {
                    Thread.sleep(250);
                } catch (InterruptedException e) {
                    System.out.println("Dashboard Error: " + e.toString());
                }
            }
        }
    }

    public Dashboard() {
        enabled = false;
        task = new DashboardThread(this);
        task.setPriority(Thread.MIN_PRIORITY);
        task.start();
    }

    public void start() {
        enabled = true;
    }

    public boolean enabled() {
        return enabled;
    }

    public void stop() {
        enabled = false;
    }
    
    public void setGyro(Gyro gyro)
    {
    	this.gyro = gyro;
    }
    
    public void setLiftMotor(CANJaguar lift) {
    	this.lift = lift;
    }
    
    public void setLiftEncoder(Encoder liftEncoder) {
    	this.liftEncoder = liftEncoder;
    }
    
    public void setPDP(PowerDistributionPanel pdp) {
    	this.pdp = pdp;
    }
    
    public int getAutoMode() {
    	return (int)SmartDashboard.getNumber("Auto Mode", 0);
    }
    public int getDriveMode() {
    	return (int)SmartDashboard.getNumber("Drive Mode", 0);
    }
}
