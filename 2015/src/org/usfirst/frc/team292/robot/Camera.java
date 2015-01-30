package org.usfirst.frc.team292.robot;

import com.ni.vision.NIVision;
import com.ni.vision.NIVision.*;

import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.image.*;
import edu.wpi.first.wpilibj.vision.AxisCamera;

public class Camera extends java.lang.Thread {
    static final boolean cameraAttached = true;
    static final int kImageHeightMin = 0;
    static final int kImageHeightMax = 200;
    static final int kImageWidthMin = 0;
    static final int kImageWidthMax = 200;
	static final int redLow = 150;
	static final int redHigh = 255;
	static final int greenLow = 150;
	static final int greenHigh = 255;
	static final int blueLow = 0;
	static final int blueHigh = 120;
	
    CameraServer server;
    int session;
    Image frame;
    NIVision.Rect rect;
    boolean lightState;
    
    AxisCamera cam;
    private ParticleFilterCriteria2[] cc = {new ParticleFilterCriteria2(), new ParticleFilterCriteria2()};
    
	public Camera() {
		server = CameraServer.getInstance();
		server.setQuality(50);
		
		cam = new AxisCamera("10.2.92.40");
        cam.writeResolution(AxisCamera.Resolution.k320x240);
        cam.writeCompression(30);
        cc[0] = new ParticleFilterCriteria2(NIVision.MeasurementType.MT_BOUNDING_RECT_WIDTH, 
                kImageWidthMin, kImageWidthMax, 0, 0);
        cc[1] = new ParticleFilterCriteria2(NIVision.MeasurementType.MT_BOUNDING_RECT_HEIGHT, 
        		kImageHeightMin, kImageHeightMax, 0, 0);
        
        frame = NIVision.imaqCreateImage(NIVision.ImageType.IMAGE_RGB, 0);
	}
	
	public void Flashy(boolean input){
		lightState = input;
	}
	
 	public void run() {
		while(true)
		{
			try {
				ImageBase image = getFilteredImg();
		        
				if(image != null) {
					server.setImage(image.image);
				}
			} catch (NIVisionException e) {
				e.printStackTrace();
			}
//	        NIVision.IMAQdxGrab(session, frame, 1);
	        
//	        NIVision.imaqWriteFile(frame, "image", NIVision.RGB_TRANSPARENT);
//	        
//	        RGBImage image = null;
//	        BinaryImage binaryImage = null;
//	        try {
//	        	image = new RGBImage("image");
//	        	
//	        	binaryImage = image.thresholdRGB(redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);
//	        } catch (NIVisionException e) {
//	        	System.out.println("ERROR " + e);
//	        }
	        
	        if(DriverStation.getInstance().isOperatorControl() && DriverStation.getInstance().isEnabled())
	        {
//	            NIVision.imaqColorThreshold(frame, frame, 255, ColorMode.RGB, new NIVision.Range(redLow,redHigh), new NIVision.Range(greenLow,greenHigh), new NIVision.Range(blueLow,blueHigh));
	            
	          {
	           
	            	
	            	
	            	
	            
	        	  
	        	  if(lightState == true){
	        		 rect = new NIVision.Rect (305, 10, 100, 100);
	            	NIVision.imaqDrawShapeOnImage (frame, frame, rect,
	            			DrawMode.PAINT_VALUE, ShapeMode.SHAPE_OVAL, 10000.0f); 
	        	  }
	        	  
	        	  rect = new NIVision.Rect(100, 10, 100, 100);
	            	NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	            			DrawMode.PAINT_VALUE, ShapeMode.SHAPE_OVAL, 2500.0f);
	            	
	            	rect = new NIVision.Rect(203, 10, 100, 100);
	            	NIVision.imaqDrawShapeOnImage(frame, frame, rect,
	            			DrawMode.PAINT_VALUE, ShapeMode.SHAPE_OVAL, 354666666.0f);
	            	
	            	
	            }
	        }
		}
	}
 	
    public ImageBase getFilteredImg() throws NIVisionException {
        if (cam.isFreshImage()) {
        	ColorImage image = cam.getImage();
            //BinaryImage thresholdImage = image.thresholdRGB(redLow, redHigh, greenLow, greenHigh, blueLow, blueHigh);
            //BinaryImage bigObjectsImage = thresholdImage.removeSmallObjects(false, 2);
            //BinaryImage convexHullImage = bigObjectsImage.convexHull(false);
            //BinaryImage filteredImage = convexHullImage.particleFilter(cc);
            //convexHullImage.free();
            //thresholdImage.free();
            //image.free();
            //bigObjectsImage.free();
            return (image);
        } else {
            return(null);
        }
    }
}
