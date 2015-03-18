package org.usfirst.frc.team292.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.vision.USBCamera;

public class Camera extends java.lang.Thread {
    CameraServer server;
    USBCamera topCam, bottomCam;
    boolean topCameraSelected = false;
    boolean topCameraStarted = false;
    boolean bottomCameraStarted = false;
    
	public Camera(String bottomCameraName, String topCameraName) {
		server = CameraServer.getInstance();
		server.setQuality(50);
		
		topCam = new USBCamera(topCameraName);
		if(topCameraName.equals(bottomCameraName)) {
			bottomCam = topCam;
		} else {
			bottomCam = new USBCamera(bottomCameraName);
		}
	}
	
 	public void run() {
 	    Image image = NIVision.imaqCreateImage(ImageType.IMAGE_RGB, 0);
		while(true)
		{
			if(topCameraSelected) {
				if(bottomCameraStarted) {
					bottomCam.stopCapture();
					bottomCameraStarted = false;
				}
				if(!topCameraStarted) {
					topCam.startCapture();
					topCameraStarted = true;
				}
				topCam.getImage(image);
			} else {
				if(topCameraStarted) {
					topCam.stopCapture();
					topCameraStarted = false;
				}
				if(!bottomCameraStarted) {
					bottomCam.startCapture();
					bottomCameraStarted = true;
				}
				bottomCam.getImage(image);
			}
			server.setImage(image);
		}
	}
 	
 	public void viewTopCamera() {
 		topCameraSelected = true;
 	}
 	
 	public void viewBottomCamera() {
 		topCameraSelected = false;
 	}
}
