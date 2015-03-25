package org.usfirst.frc.team292.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Gyro;
import edu.wpi.first.wpilibj.PowerDistributionPanel;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx_mxp.AHRS;

public class Dashboard {

    private boolean enabled;
    private Thread task;
    private boolean run = true;
    private Gyro gyro;
    private CANTalon lift;
    private Encoder liftEncoder;
    private PowerDistributionPanel pdp;
    private AHRS navx;

    private class DashboardThread extends Thread {

        Dashboard db;

        DashboardThread(Dashboard db) {
            this.db = db;
        }

        public void run() {
            while (run) {
                if (db.enabled()) {
                	if(gyro != null) SmartDashboard.putNumber("Analog Gyro", gyro.getAngle());
                	if(liftEncoder != null) SmartDashboard.putNumber("Lift", liftEncoder.get());
                	if(lift != null) SmartDashboard.putBoolean("Lift At Bottom", !lift.isFwdLimitSwitchClosed());
                	if(lift != null) SmartDashboard.putBoolean("Lift At Top", !lift.isRevLimitSwitchClosed());
                	//if(pdp != null) pdp.updateTable();
                	//if(pdp != null) SmartDashboard.putData("PDP", pdp);
					if (navx != null) {
						SmartDashboard.putNumber("Gyro", navx.getYaw());
						SmartDashboard.putBoolean("NavX_Connected",
								navx.isConnected());
						SmartDashboard.putBoolean("NavX_IsCalibrating",
								navx.isCalibrating());
						SmartDashboard.putNumber("NavX_Yaw", navx.getYaw());
						SmartDashboard.putNumber("NavX_Pitch", navx.getPitch());
						SmartDashboard.putNumber("NavX_Roll", navx.getRoll());
						SmartDashboard.putNumber("NavX_CompassHeading",
								navx.getCompassHeading());
						SmartDashboard.putNumber("NavX_Update_Count",
								navx.getUpdateCount());
						SmartDashboard.putNumber("NavX_Byte_Count",
								navx.getByteCount());
						SmartDashboard.putNumber("NavX_Accel_X",
								navx.getWorldLinearAccelX());
						SmartDashboard.putNumber("NavX_Accel_Y",
								navx.getWorldLinearAccelY());
						SmartDashboard.putNumber("NavX_Accel_Z",
								navx.getWorldLinearAccelZ());
						SmartDashboard.putBoolean("NavX_IsMoving",
								navx.isMoving());
						SmartDashboard.putNumber("NavX_Temp_C", navx.getTempC());
						SmartDashboard.putNumber("NavX_Velocity_X",
								navx.getVelocityX());
						SmartDashboard.putNumber("NavX_Velocity_Y",
								navx.getVelocityY());
						SmartDashboard.putNumber("NavX_Displacement_X",
								navx.getDisplacementX());
						SmartDashboard.putNumber("NavX_Displacement_Y",
								navx.getDisplacementY());
						SmartDashboard.putNumber("NavX_Magnetometer_X",
								navx.getCalibratedMagnetometerX());
						SmartDashboard.putNumber("NavX_Magnetometer_Y",
								navx.getCalibratedMagnetometerY());
						SmartDashboard.putNumber("NavX_Magnetometer_Z",
								navx.getCalibratedMagnetometerZ());
						SmartDashboard.putBoolean("NavX_Magnetometer_Calibrated",
								navx.isMagnetometerCalibrated());
					}
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
    
    public void setLiftMotor(CANTalon lift) {
    	this.lift = lift;
    }
    
    public void setLiftEncoder(Encoder liftEncoder) {
    	this.liftEncoder = liftEncoder;
    }
    
    public void setPDP(PowerDistributionPanel pdp) {
    	this.pdp = pdp;
    }
    
    public void setNavX(AHRS navx) {
    	this.navx = navx;
    }
    
    public int getAutoMode() {
    	return (int)SmartDashboard.getNumber("Auto Mode", 0);
    }
    
    public int getDriveMode() {
    	return (int)SmartDashboard.getNumber("Drive Mode", 0);
    }
}