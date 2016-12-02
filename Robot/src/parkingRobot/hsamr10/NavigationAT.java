package parkingRobot.hsamr10;

import lejos.geom.Point;
import lejos.nxt.LCD;
import lejos.geom.Line;
import lejos.robotics.navigation.Pose;

import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import parkingRobot.hsamr10.NavigationThread;

import parkingRobot.hsamr10.Navigation.StraightLine;

import java.util.LinkedList;
import java.util.List;


/**
 * A executable basic example implementation of the corresponding interface provided by the Institute of Automation with
 * limited functionality:
 * <p>
 * In this example only the both encoder sensors from the {@link IPerception} implementation are used for periodically calculating
 * the robots position and corresponding heading angle (together called 'pose'). Neither any use of the map information or other
 * perception sensor information is used nor any parking slot detection is performed, although the necessary members are already
 * prepared. Advanced navigation calculation and parking slot detection should be developed and invented by the students.  
 * 
 * @author IfA
 */
public class NavigationAT implements INavigation{
	
	/**
	 * line information measured by right light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorRight	=	0;
	/**
	 * line information measured by left light sensor: 0 - beside line, 1 - on line border or gray underground, 2 - on line
	 */
	int lineSensorLeft	=	0;
	
	/**
	 * reference to {@link IPerception.EncoderSensor} class for left robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderLeft							=	null;
	/**
	 * reference to {@link IPerception.EncoderSensor} class for right robot wheel which measures the wheels angle difference
	 * between actual an last request
	 */
	IPerception.EncoderSensor encoderRight							=	null;
	
	/**
	 * reference to data class for measurement of the left wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementLeft 	= 	null;
	/**
	 * reference to data class for measurement of the right wheels angle difference between actual an last request and the
	 * corresponding time difference
	 */
	IPerception.AngleDifferenceMeasurement angleMeasurementRight	= 	null;
	
	/**
	 * reference to {@link IPerception.OdoSensor} class for mouse odometry sensor to measure the ground displacement
	 * between actual an last request
	 */
	IPerception.OdoSensor mouseodo = null;	
		
	/**
	 * reference to data class for measurement of the mouse odometry sensor to measure the ground displacement between
	 * actual an last request and the corresponding time difference
	 */
	IPerception.OdoDifferenceMeasurement mouseOdoMeasurement = null;
	
	/**
	 * distance from optical sensor pointing in driving direction to obstacle in mm
	 */
	double frontSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the front)
	 */
	double frontSideSensorDistance	=	0;
	/**
	 * distance from optical sensor pointing in opposite of driving direction to obstacle in mm
	 */
	double backSensorDistance		=	0;
	/**
	 * distance from optical sensor pointing to the right side of robot to obstacle in mm (sensor mounted at the back)
	 */
	double backSideSensorDistance	=	0;
	
	double angle_by_DistanceSensors = 666*Math.PI/180;

	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.0278; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	LEFT_WHEEL_RADIUS * 3740/3721; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.096; // only rough guess, to be measured exactly and maybe refined by experiments
	
	/**
	 * map array of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	//Line[] map 								= null;
	private LinkedList<StraightLine> map = null;
	public 	StraightLine currentLine	 = null;
	/**
	 * defines current line of map the module is operating on
	 */
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        		= null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn		= false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose								= new Pose();
	
	boolean detectingParkingSlot		 = false;
	double backBoundaryPosition			 = 0;
	int parkingSlotIDCounter			 = 0;
	static final double DISTANCE_ROBOTCOURSE_TO_PARKINGAREA = 0.30;
	
	/**
	 * thread started by the 'Navigation' class for background calculating
	 */
	NavigationThread navThread = new NavigationThread(this);

	
	/**
	 * provides the reference transfer so that the class knows its corresponding perception object (to obtain the measurement
	 * information from) and starts the navigation thread.
	 * 
	 * @param perception corresponding main module Perception class object
	 * @param monitor corresponding main module Monitor class object
	 */
	public NavigationAT(IPerception perception, IMonitor monitor){
		this.perception   = perception;
		this.monitor 	  = monitor;
		this.encoderLeft  = perception.getNavigationLeftEncoder();
		this.encoderRight = perception.getNavigationRightEncoder();
		this.mouseodo	  = perception.getNavigationOdo();
		
		
		monitor.addNavigationVar("xCoordinate");
		monitor.addNavigationVar("yCoordinate");
		monitor.addNavigationVar("angle");
		monitor.addNavigationVar("angle2");
		monitor.addNavigationVar("map");
		monitor.addNavigationVar("frontSide");
		monitor.addNavigationVar("backSide");
		monitor.addNavigationVar("front");
		monitor.addNavigationVar("back");
		
		
		/*
		monitor.addNavigationVar("xCoordinate");
		monitor.addNavigationVar("yCoordinate");
		monitor.addNavigationVar("angle");
		monitor.addNavigationVar("leftEncoder");
		monitor.addNavigationVar("rightEncoder");
		monitor.addNavigationVar("leftTime");
		monitor.addNavigationVar("rightTime");	
		*/
		navThread.setPriority(Thread.MAX_PRIORITY - 1);
		navThread.setDaemon(true); // background thread that is not need to terminate in order for the user program to terminate
		navThread.start();
	}
	
	
	// Inputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setMap(lejos.geom.Line[])
	 */
	public void setMap(Line[] map){
		this.map = new LinkedList<StraightLine>();
		this.currentLine = new StraightLine( map[0] );
		this.map.add( this.currentLine );
		for (int i=1; i<map.length; i++) {
			this.map.add(new StraightLine( map[i] ) );
			this.map.get( i-1 ).setFollowingLine( this.map.get(i) );
		}
		this.map.get( map.length-1 ).setFollowingLine( this.currentLine );
		
		for (int i=0; i<map.length; i++) {
			this.map.get(i).id = i;
			this.map.get(i).monitor = this.monitor;
		}
		
	}
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#setDetectionState(boolean)
	 */
	public void setDetectionState(boolean isOn){
		this.parkingSlotDetectionIsOn = isOn;
	}
	
	
	// 	Class control
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#updateNavigation()
	 */
	public synchronized void updateNavigation(){	
		this.updateSensors();
		this.calculateLocation();
		if (this.parkingSlotDetectionIsOn)
				this.detectParkingSlot();
		
		// MONITOR (example)
//		monitor.writeNavigationComment("Navigation");
		
		

		if (this.map != null) {
			
			monitor.writeNavigationVar("xCoordinate", ""+( this.pose.getX()*100 ) );
			monitor.writeNavigationVar("yCoordinate", ""+( this.pose.getY()*100 ) );
			monitor.writeNavigationVar("angle", ""+( this.pose.getHeading() *180/Math.PI ) );
			monitor.writeNavigationVar("angle2", ""+this.angle_by_DistanceSensors * 180/Math.PI);
			monitor.writeNavigationVar("map", ""+ this.currentLine.id);
			monitor.writeNavigationVar("frontSide", ""+this.frontSideSensorDistance);
			monitor.writeNavigationVar("backSide", ""+this.backSideSensorDistance);
			monitor.writeNavigationVar("front", ""+this.frontSensorDistance);
			monitor.writeNavigationVar("back", ""+this.backSensorDistance);
			/*
			LCD.clear();
			
			LCD.drawString("X (in cm): " + (this.getPose().getX()*100), 0, 0);
			LCD.drawString("Y (in cm): " + (this.getPose().getY()*100), 0, 1);
			LCD.drawString("Phi (grd): " + (this.getPose().getHeading()/Math.PI*180), 0, 2);
			LCD.drawString("Line " + this.currentLine.id, 0, 3);
			LCD.drawString("Parking Slots: " + this.currentLine.getParkingSlots().size(), 0, 4);
			LCD.drawString("FrontSide " + this.frontSideSensorDistance, 0, 5);
			LCD.drawString("BackSide " + this.backSideSensorDistance, 0, 6);
			LCD.drawString("Angle " + this.angle_by_DistanceSensors * 180/Math.PI, 0, 7);
			//LCD.drawString(""+this.currentLine.getDirection(), 0, 7);
			//LCD.drawString("Line " + this.currentLine.id, 0, 7);
			/*LCD.drawString("Angle " + this.currentLine.conditionAngle, 0, 1);
			LCD.drawString("Distance " + this.currentLine.conditionDistance, 0, 2);*/
			//perception.showSensorData();

		}

		/*
		LCD.clear();
		LCD.drawString("Linie: " + this.currentLine.id , 0, 0);
		*/
	}
	
	
	// Outputs
	
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getPose()
	 */
	public synchronized Pose getPose(){
		return this.pose;
	}
	/* (non-Javadoc)
	 * @see parkingRobot.INavigation#getParkingSlots()
	 */
	public synchronized ParkingSlot[] getParkingSlots() {
		List<ParkingSlot> parkingSlots = new LinkedList<ParkingSlot>();
		for (int i=0; i<this.map.size(); i++) {
			parkingSlots.addAll( this.map.get(i).getParkingSlots() );
		}
		
		ParkingSlot[] parkingSlotsArr = new ParkingSlot[parkingSlots.size()];
		for (int i=0; i<parkingSlots.size(); i++) {
			parkingSlotsArr[i] = parkingSlots.get(i);
		}
		return parkingSlotsArr;
	}
	
	public int getLineID() {
		return this.currentLine.id;
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){		
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance= perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		Pose poseIncrementalEncoder = calculateLocationIncrementalEncoder();
		// Pose poseMouseSensor		= calculateLocationMouseSensor(); // noch nicht implementiert
		
		// Fusion Odometriedaten
		this.pose.setLocation( poseIncrementalEncoder.getLocation() );
		this.pose.setHeading(  poseIncrementalEncoder.getHeading() );
		
		// Korrektur
		
		if (this.map != null) {
			// this.currentLine.poseCorrection( this.pose, this.perception);		
			this.currentLine = this.currentLine.update( this.pose );
		}
		
		if ( this.frontSideSensorDistance!=0 && this.frontSideSensorDistance!=Double.POSITIVE_INFINITY &&
				this.backSideSensorDistance!=0 && this.backSideSensorDistance!=Double.POSITIVE_INFINITY ) {
			double offset = 0;
			switch ( this.currentLine.getDirection() ) {
			case EAST:
				offset = Math.PI * 0 / 2;
				break;
			case NORTH:
				offset = Math.PI * 1 / 2;
				break;			
			case WEST:
				offset = Math.PI * 2 / 2;
				break;
			case SOUTH:
				offset = Math.PI * 3 / 2;
				break;
			}
			this.angle_by_DistanceSensors = Math.atan(  (this.frontSideSensorDistance - this.backSideSensorDistance) / 17.5  ) + offset; // Distance between both sensors
			
			//if ( Math.abs( this.angleMeasurementLeft.getAngleSum() - this.angleMeasurementRight.getAngleSum() )/ ((double)this.angleMeasurementLeft.getDeltaT()/1000) > 50 ||
				//	this.pose.getHeading() % (Math.PI/4) > Math.PI/6 ) {
			if ( this.pose.getX()==0 && this.pose.getY()==0 ) {
				this.pose.setHeading( (float)this.angle_by_DistanceSensors );
			}
		}
				
		
	}
	
	private Pose calculateLocationIncrementalEncoder(){
		double leftAngleSpeed 	= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
		double rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds

		double vLeft		= (leftAngleSpeed  * Math.PI * LEFT_WHEEL_RADIUS ) / 180 ; //velocity of left  wheel in m/s
		double vRight		= (rightAngleSpeed * Math.PI * RIGHT_WHEEL_RADIUS) / 180 ; //velocity of right wheel in m/s		
		double w 			= (vRight - vLeft) / WHEEL_DISTANCE; //angular velocity of robot in rad/s
		
		Double R 			= new Double(( WHEEL_DISTANCE / 2 ) * ( (vLeft + vRight) / (vRight - vLeft) ));								
		
		double ICCx 		= 0;
		double ICCy 		= 0;

		double xResult 		= 0;
		double yResult 		= 0;
		double angleResult 	= 0;
		
		double deltaT       = ((double)this.angleMeasurementLeft.getDeltaT())/1000;
		
		if (R.isNaN()) { //robot don't move
			xResult			= this.pose.getX();
			yResult			= this.pose.getY();
			angleResult 	= this.pose.getHeading();
		} else if (R.isInfinite()) { //robot moves straight forward/backward, vLeft==vRight
			xResult			= this.pose.getX() + vLeft * Math.cos(this.pose.getHeading()) * deltaT;
			yResult			= this.pose.getY() + vLeft * Math.sin(this.pose.getHeading()) * deltaT;
			angleResult 	= this.pose.getHeading();
		} else {			
			ICCx = this.pose.getX() - R.doubleValue() * Math.sin(this.pose.getHeading());
			ICCy = this.pose.getY() + R.doubleValue() * Math.cos(this.pose.getHeading());
		
			xResult 		= Math.cos(w * deltaT) * (this.pose.getX()-ICCx) - Math.sin(w * deltaT) * (this.pose.getY() - ICCy) + ICCx;
			yResult 		= Math.sin(w * deltaT) * (this.pose.getX()-ICCx) + Math.cos(w * deltaT) * (this.pose.getY() - ICCy) + ICCy;
			angleResult 	= this.pose.getHeading() + w * deltaT;
		}
		angleResult = ( angleResult + 2*Math.PI) % (2*Math.PI);
		
		Pose pose = new Pose();
		pose.setLocation((float)xResult, (float)yResult);
		pose.setHeading((float)angleResult);
		/*
		monitor.writeNavigationVar("xCoordinate", ""+( xResult ) );
		monitor.writeNavigationVar("yCoordinate", ""+( yResult ) );
		monitor.writeNavigationVar("angle", ""+( angleResult *180/Math.PI ) );
		monitor.writeNavigationVar("leftEncoder", ""+leftAngleSpeed);
		monitor.writeNavigationVar("rightEncoder", ""+rightAngleSpeed);
		monitor.writeNavigationVar("leftTime", ""+deltaT);
		monitor.writeNavigationVar("rightTime", ""+ ((double)this.angleMeasurementRight.getDeltaT())/1000);
		*/
		return pose;
	}
	
	private Pose calculateLocationMouseSensor() {
		Pose pose = new Pose();
		return pose;
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		//double x = this.pose.getX();
		//double y = this.pose.getY();
		/*
		int ID;
		Point backBoundaryPosition = new Point(0, 0);
		ParkingSlotStatus parkingSuitable;
		*/
		
		if (this.detectingParkingSlot) {
			
			
			
			if (this.frontSideSensorDistance != Double.POSITIVE_INFINITY) { // Wert anpassen
				
				if ( this.currentLine.newParkingSlotSpotted(parkingSlotIDCounter, calcParkingSlotLocation( this.currentLine ), this.backBoundaryPosition) ) {
					this.parkingSlotIDCounter++;
				}
				
				this.detectingParkingSlot = false;
				
			}
		}
		else {
			if ( this.frontSideSensorDistance == Double.POSITIVE_INFINITY &&
					this.backSideSensorDistance == Double.POSITIVE_INFINITY ) {
				
				this.detectingParkingSlot = true;
				this.backBoundaryPosition = calcParkingSlotLocation( this.currentLine );
				
			}
			
		}
		
		
	}
	
	// functions
	/**
	 * Calculates the angle of a given line. A single line is given as vector. 
	 * @param line
	 * @return angle
	 */
	private float calculateAngle(Line line){
		double x = line.getX2() - line.getX1();
		double y = line.getY2() - line.getY1();
		float angle = (float) (Math.atan2(y, x) );
		return angle;
	}	

	
	private Point calcPointOnStraightLine(double x, double y, Line line){
		Point point = new Point(0, 0);
		if ( line.getX1()==line.getX2() ) {
			point.setLocation( line.getX1(), y );
		}
		else if ( line.getY1()==line.getY2() ) {
			point.setLocation( x, line.getY1() );
		}
		return point;
	}
	
	private Point calcPointOnStraightLine(Pose pose, Line line){
		return calcPointOnStraightLine( pose.getX(), pose.getY(), line );
	}
	
	
	private double calcParkingSlotLocation(StraightLine line) {
		double coordinate = 0;
		switch ( line.getDirection() ) {
			case NORTH:
				coordinate = this.pose.getY() + DISTANCE_ROBOTCOURSE_TO_PARKINGAREA * Math.tan( this.pose.getHeading() - Math.PI/2 );
				break;
			case SOUTH:
				coordinate = this.pose.getY() - DISTANCE_ROBOTCOURSE_TO_PARKINGAREA * Math.tan( this.pose.getHeading() - Math.PI/2 );
				break;
			case EAST:
				coordinate = this.pose.getX() + DISTANCE_ROBOTCOURSE_TO_PARKINGAREA * Math.tan( this.pose.getHeading() );
				break;
			case WEST:
				coordinate = this.pose.getX() - DISTANCE_ROBOTCOURSE_TO_PARKINGAREA * Math.tan( this.pose.getHeading() );
				break;

		}
		return coordinate;
	}
}
