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
import parkingRobot.hsamr10.Navigation.StraightLine.Direction;

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
	
	static private final int LENGTH_MOVING_AVERAGE = 5;
	
	static private final double DISTANCE_BLACKLINE_LINESENSOR = 0.01;
	private int filterID 			= 0; // counter for array, floating average calculation
	private int[] leftLine			= new int[LENGTH_MOVING_AVERAGE];
	private int[] rightLine			= new int[LENGTH_MOVING_AVERAGE];
	private float leftLineAverage 	= 0;
	private float rightLineAverage	= 0;
	
	
	private Pose lastPose = new Pose();
	private int lockAngleCorrection = 0;
	
	private boolean lineFollowerActive = true;
	
	
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
	
	private double lastFrontSensorDistance = 0;
	private double lastBackSensorDistance  = 0;

	/**
	 * robot specific constant: radius of left wheel
	 */
	static final double LEFT_WHEEL_RADIUS	= 	0.0276; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: radius of right wheel
	 */
	static final double RIGHT_WHEEL_RADIUS	= 	LEFT_WHEEL_RADIUS * 3740/3721; // only rough guess, to be measured exactly and maybe refined by experiments
	/**
	 * robot specific constant: distance between wheels
	 */
	static final double WHEEL_DISTANCE		= 	0.096; // only rough guess, to be measured exactly and maybe refined by experiments
	
	private double leftAngleSpeed		 = 0;
	private double rightAngleSpeed		 = 0;
	
	/**
	 * map list of line references, whose corresponding lines form a closed chain and represent the map of the robot course
	 */
	//Line[] map 								= null;
	private LinkedList<StraightLine> map = null;
	/**
	 * defines current line of map the module is operating on
	 */
	private	StraightLine currentLine	 = null;	
	/**
	 * reference to the corresponding main module Perception class
	 */
	IPerception perception 	        	 = null;
	/**
	 * reference to the corresponding main module Monitor class
	 */
	IMonitor monitor					 = null;
	/**
	 * indicates if parking slot detection should be switched on (true) or off (false)
	 */
	boolean parkingSlotDetectionIsOn	 = false;
	/**
	 * pose class containing bundled current X and Y location and corresponding heading angle phi
	 */
	Pose pose							 = new Pose();
	/**
	 * variable, to check whether a parking slot measuring process is ongoing / a parking slot's beginning has been spotted
	 */
	static final int DETECTION_DELAY			= 3;
	private boolean detectingSlotFrontSide		= false;
	private boolean detectingSlotBackSide		= false;
	private int detectionDelayFrontSide			= 0;
	private int detectionDelayBackSide			= 0;
	/**
	 * variable of the front side sensor to save the parking slot's boundaries temporarily
	 * 0... front boundary
	 * 1... back  boundary
	 * 2... front boundary measurement quality
	 * 3... back  boundary measurement quality
	 */
	private double[] slotBoundariesFrontSide	= new double[4];
	/**
	 * variable of the back side sensor to save the parking slot's boundaries temporarily
	 * 0... front boundary
	 * 1... back  boundary
	 * 2... front boundary measurement quality
	 * 3... back  boundary measurement quality
	 */
	private double[] slotBoundariesBackSide		= new double[4];
	/**
	 * quality due to sensor's reaction time, evaluated by the robot's speed
	 */
	private double distanceSensorQuality		= 0;
	

	/**
	 * ID, that is given to new parking slots. Increments with each new slot. Once an ID has been used, it cannot be used, even if 
	 * the corresponding slot is deleted afterwards.
	 */
	private int parkingSlotIDCounter			= 0;
	private int lastSlotChangedID				= 0;
	
	
	static private final double OFFSET_PARKSLOT_BACKSIDE  = -0.100;
	static private final double OFFSET_PARKSLOT_FRONTSIDE =  0.075;
	/**
	 * distance from the line course to a parking slot parallel to the line. Used for angle correction.
	 */
	static final double DISTANCE_ROBOTCOURSE_TO_BORDER = 0.20;
	static final double SIDESENSOR_DISTANCE = OFFSET_PARKSLOT_FRONTSIDE - OFFSET_PARKSLOT_BACKSIDE;
	
	/**
	 * accuracy of pose, park slot measurements
	 */
	
	
	static private final double RELATIVEDEVIATION_ALONG_ROAD_RADENC		= 0.008; // deviation in driving direction
	static private final double RELATIVEDEVIATION_NEXTTO_ROAD_RADENC	= 0.020; // deviation vertical to driving direction
	static private final double RELATIVEDEVIATION_ANGLE_RADENC			= 0.010; 
	
	private double xDeviation		= 0;
	private double yDeviation		= 0;
	private double angleDeviation	= 0;
	
	
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
		
		
		monitor.addNavigationVar("xCoordinate" );
		monitor.addNavigationVar("yCoordinate");
		monitor.addNavigationVar("angle" );

		monitor.addNavigationVar("frontSide");
		monitor.addNavigationVar("backSide");

		
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
		
		if (!isOn) { // add finalising detection
			if (this.detectingSlotBackSide) {
				this.detectingSlotBackSide = false;
			}
			this.detectingSlotFrontSide = false;
		}
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
		
			
			LCD.clear();
			LCD.drawString("B: " + slotBoundariesBackSide[1], 0, 0);
			LCD.drawString("F: " + slotBoundariesBackSide[0], 0, 1);
			LCD.drawString("DevB: " + slotBoundariesBackSide[3], 0, 2);
			LCD.drawString("DevF: " + slotBoundariesBackSide[2], 0, 3);
			LCD.drawString("FSide: " + this.frontSideSensorDistance, 0, 5);
			
			monitor.writeNavigationVar("xCoordinate", ""+this.pose.getX()*100 );
			monitor.writeNavigationVar("yCoordinate", ""+this.pose.getY()*100);
			monitor.writeNavigationVar("angle", ""+this.pose.getHeading()*180/Math.PI );

			monitor.writeNavigationVar("frontSide", ""+this.frontSideSensorDistance);
			monitor.writeNavigationVar("backSide", ""+this.backSideSensorDistance);

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
	public synchronized Pose getPose(){ // defines new object, in order to remain unchanged in other modules
		Pose posecopy = new Pose();
		float x, y;
		x = this.pose.getX();
		y = this.pose.getY();
		posecopy.setLocation(x, y);
		posecopy.setHeading( this.pose.getHeading() );
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
	
	public boolean detectingParkingSlot() {
		return this.detectingSlotBackSide || this.detectingSlotFrontSide;
	}
	
	public int getLastChangedSlot() {
		return this.lastSlotChangedID;  
	}
	
	public void setLineFollowerState(boolean state) {
		this.lineFollowerActive = state;
	}
	
	
	// Private methods
	
	/**
	 * calls the perception methods for obtaining actual measurement data and writes the data into members
	 */
	private void updateSensors(){	
		/*
		this.lineSensorRight		= perception.getRightLineSensor();
		this.lineSensorLeft  		= perception.getLeftLineSensor();
		*/
		// Achtung, ggf. aendern sich diese Methoden wieder.
		
		this.lineSensorRight		= perception.getRightLineSensorValue();
		this.lineSensorLeft			= perception.getLeftLineSensorValue();
		
		this.angleMeasurementLeft  	= this.encoderLeft.getEncoderMeasurement();
		this.angleMeasurementRight 	= this.encoderRight.getEncoderMeasurement();

		this.mouseOdoMeasurement	= this.mouseodo.getOdoMeasurement();

		this.frontSensorDistance	= perception.getFrontSensorDistance();
		this.frontSideSensorDistance= perception.getFrontSideSensorDistance();
		this.backSensorDistance		= perception.getBackSensorDistance();
		this.backSideSensorDistance	= perception.getBackSideSensorDistance();
		
		this.leftAngleSpeed		= this.angleMeasurementLeft.getAngleSum()  / ((double)this.angleMeasurementLeft.getDeltaT()/1000);  //degree/seconds
	 	this.rightAngleSpeed 	= this.angleMeasurementRight.getAngleSum() / ((double)this.angleMeasurementRight.getDeltaT()/1000); //degree/seconds
	}		 	
	
	/**
	 * calculates the robot pose from the measurements
	 */
	private void calculateLocation(){
		lastPose.setLocation( pose.getX(), pose.getY() );
		lastPose.setHeading( pose.getHeading() );
		
		Pose poseIncrementalEncoder = calculateLocationIncrementalEncoder();
		Pose poseMouseSensor		= calculateLocationMouseSensor(); // noch nicht fusioniert
		
		// Fusion Odometriedaten
		this.pose.setLocation( poseIncrementalEncoder.getLocation() );
		this.pose.setHeading(  poseIncrementalEncoder.getHeading() );
		
		// Korrektur
		
		if (this.map != null ) {
			
			if (currentLine.id%2 == 0) {
				this.xDeviation += ( Math.abs( lastPose.getX() - poseIncrementalEncoder.getX() ) * RELATIVEDEVIATION_ALONG_ROAD_RADENC );
				this.yDeviation += ( Math.abs( lastPose.getY() - poseIncrementalEncoder.getY() ) * RELATIVEDEVIATION_NEXTTO_ROAD_RADENC );
			}
			else {
				this.xDeviation += ( Math.abs( lastPose.getX() - poseIncrementalEncoder.getX() ) * RELATIVEDEVIATION_NEXTTO_ROAD_RADENC );
				this.yDeviation += ( Math.abs( lastPose.getY() - poseIncrementalEncoder.getY() ) * RELATIVEDEVIATION_ALONG_ROAD_RADENC );
			}
			this.angleDeviation += ( Math.abs( lastPose.getHeading() - poseIncrementalEncoder.getHeading() ) * RELATIVEDEVIATION_ANGLE_RADENC );
			
			
			// using back and front sensor
			
			double relativeAngle = currentLine.absAngle(pose.getHeading(), currentLine.getAngle());
			double x			 = pose.getX();
			double y			 = pose.getY();
			double xdist, ydist;
			
			if ( backSensorDistance!=Double.POSITIVE_INFINITY && false &&
					backSensorDistance != 0 && 
				relativeAngle < Math.toRadians( 15 ) &&
				lastBackSensorDistance < backSensorDistance) {
				
				switch (this.currentLine.id) {
				case 0:
					//x = ( ( -2.5 + Math.cos(relativeAngle) * backSensorDistance ) / 100 + x ) / 2;
					xdist = ( -2.5 + Math.cos(relativeAngle) * backSensorDistance ) / 100;
					ydist = y;
					break;
				case 1:
					xdist = x;
					ydist = ( -2.5 + Math.cos(relativeAngle) * backSensorDistance ) / 100;
					break;
				case 3:
					xdist = x;
					ydist = ( 62.5 - Math.cos(relativeAngle) * backSensorDistance ) / 100;
					break;
				case 6:
					xdist = ( 32.5 - Math.cos(relativeAngle) * backSensorDistance ) / 100;
					ydist = y;
					break;
				case 7:
					xdist = x;
					ydist = ( 62.5 - Math.cos(relativeAngle) * backSensorDistance ) / 100;
					break;
				default:
					xdist = x;
					ydist = y;
				}
				
				if ( this.currentLine.id%2 == 0 ) { // horizontal line
					if ( Math.abs( x-xdist ) < 0.06 ) {
						x = (x+xdist) / 2;
					}
				}
				
				if ( this.currentLine.id%2 == 1 ) { // vertical line
					if ( Math.abs( y-ydist ) < 0.06 ) {
						y = (y+ydist) / 2;
					}
				}
				
				

				this.pose.setLocation( (float)x, (float)y);
			}
			
			this.lastBackSensorDistance = backSensorDistance; 
			
						
			// add new value and remove oldest value from mean calculation (gleitender Mittelwert)
						
			if ( Math.max(leftAngleSpeed, rightAngleSpeed) > 0 ) {		// robot is moving
				this.leftLineAverage += (float)( lineSensorLeft - this.leftLine [filterID] ) / LENGTH_MOVING_AVERAGE;
				this.rightLineAverage+= (float)( lineSensorRight- this.rightLine[filterID] ) / LENGTH_MOVING_AVERAGE;
				
				this.leftLine[filterID] = lineSensorLeft;
				this.rightLine[filterID]= lineSensorRight;
				
				this.filterID = ( this.filterID+1 ) % LENGTH_MOVING_AVERAGE;
			}
			
			
			
			double angle, error;
			// maybe these variables are already initialised correctly
			x = pose.getX();
			y = pose.getY();
			if ( Math.min(leftLineAverage, rightLineAverage) > 80 && this.lineFollowerActive) {  // Robot is following line
				if 		( currentLine.getDirection()==Direction.NORTH || currentLine.getDirection()==Direction.SOUTH ) {
					
					error = x - currentLine.getConstCoordinate();
					if ( Math.abs( error ) > DISTANCE_BLACKLINE_LINESENSOR ) { // Robot must be closer than 1 cm to the line
						if (error > 0) {		x = x - 0.5 * (error - DISTANCE_BLACKLINE_LINESENSOR);	}
						else if (error < 0) {	x = x - 0.5 * (error + DISTANCE_BLACKLINE_LINESENSOR);	}
						
						
						
						angle = Math.atan2( y-lastPose.getY(), x-lastPose.getX() ) + 0 * pose.getHeading(); // hart
						
						pose.setLocation( (float) x, (float) y);
						if ( this.lockAngleCorrection==0 && currentLine.absAngle(angle, pose.getHeading()) < 10 * Math.PI/180 ) {
							this.lockAngleCorrection++;
							pose.setHeading((float)angle);
							this.angleDeviation = Math.max(this.angleDeviation- 5* Math.PI/180, 0);
						}
					}
					this.xDeviation = Math.min( this.xDeviation, Math.abs(error));
				}
				
				else if ( currentLine.getDirection()==Direction.EAST  || currentLine.getDirection()==Direction.WEST ) {
					
					error = y - currentLine.getConstCoordinate();
					if ( Math.abs( error ) > DISTANCE_BLACKLINE_LINESENSOR ) { // Robot must be closer than 1 cm to the line
						if (error > 0) {		y = y - 0.5 * (error-DISTANCE_BLACKLINE_LINESENSOR);	}
						else if (error < 0) {	y = y - 0.5 * (error+DISTANCE_BLACKLINE_LINESENSOR);	}
						angle = Math.atan2( y-lastPose.getY(), x-lastPose.getX() ) + 0 * pose.getHeading(); // hart, evtl. gefiltert?
						
						pose.setLocation( (float) x, (float) y);
						if ( this.lockAngleCorrection==0 && currentLine.absAngle(angle, pose.getHeading()) < 10 * Math.PI/180 ) {
							this.lockAngleCorrection++;
							pose.setHeading((float)angle);
							this.angleDeviation = Math.max(this.angleDeviation- 5* Math.PI/180, 0);
						}
					}
					this.yDeviation = Math.min( this.yDeviation, Math.abs(error));
				}
			}
			
			
			if (this.lockAngleCorrection > 0) { this.lockAngleCorrection++; }
			this.lockAngleCorrection = this.lockAngleCorrection % 10; // update angle only periodically
			
			
			
			// if black underground is detected, check, whether the robot is turning to a new line
			if ( Math.min(this.lineSensorLeft, this.lineSensorRight) < 20) {
				int previousLineID = currentLine.id;
				
				this.currentLine = this.currentLine.update( this.pose );
				
				if ( previousLineID != currentLine.id ) {
					this.detectingSlotBackSide = false;
					this.detectingSlotFrontSide= false;
					if ( previousLineID % 2 == 0 ) {  this.yDeviation = Math.min(this.yDeviation, 0.01);  }
					else {  this.yDeviation = Math.min(this.xDeviation, 0.01);  }
				}
			}
			 
		}
		
		if ( this.frontSideSensorDistance!=0 && this.frontSideSensorDistance!=Double.POSITIVE_INFINITY &&
				this.backSideSensorDistance!=0 && this.backSideSensorDistance!=Double.POSITIVE_INFINITY &&
				leftAngleSpeed==0 && rightAngleSpeed==0) {
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
			double angle_by_DistanceSensors = Math.atan(  (this.frontSideSensorDistance - this.backSideSensorDistance) / SIDESENSOR_DISTANCE  ) + offset;
			
			if ( currentLine.absAngle( angle_by_DistanceSensors, pose.getHeading()) < 20 * Math.PI/180 ) {
				this.pose.setHeading( (float) angle_by_DistanceSensors );
				this.angleDeviation = Math.min( this.angleDeviation, Math.atan(1.0 / SIDESENSOR_DISTANCE)); // distance sensors' resolution is 1 cm
			}

			
		}
		
		
				
		
	}
	/**
	 * calculates robot pose from via incremental encoders (odometry)
	 * 
	 * @return pose class returns robot pose
	 */
	private Pose calculateLocationIncrementalEncoder(){
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
		angleResult = ( angleResult + 0*2*Math.PI) % (2*Math.PI); // nur Werte zwischen 0... 360°
		
		Pose pose = new Pose();
		pose.setLocation((float)xResult, (float)yResult);
		pose.setHeading( (float)angleResult);
		
		/*
		monitor.writeNavigationVar("xCoordinate", ""+( xResult ) );
		monitor.writeNavigationVar("yCoordinate", ""+( yResult ) );
		monitor.writeNavigationVar("angle", ""+( angleResult *180/Math.PI ) );
		monitor.writeNavigationVar("leftEncoder", ""+leftAngleSpeed);
		monitor.writeNavigationVar("rightEncoder", ""+rightAngleSpeed);
		monitor.writeNavigationVar("leftTime", ""+deltaT);		
		*/
		return pose;
	}
	
	private Pose calculateLocationMouseSensor() {
		Pose pose = new Pose();
		double uShift = 0.001 * this.mouseOdoMeasurement.getUSum();
		double vShift = 0.001 * this.mouseOdoMeasurement.getVSum();
		double deltaT = 0.001 * this.mouseOdoMeasurement.getDeltaT();
		
		double angle  = this.pose.getHeading();
		
		double x	 = this.pose.getX() + uShift * Math.cos( angle ) - vShift * Math.sin( angle );
		double y	 = this.pose.getY() + uShift * Math.sin( angle ) + vShift * Math.cos( angle );
		angle = this.pose.getHeading() + Math.atan2(vShift, uShift);
		
		pose.setLocation((float)x, (float)y);
		pose.setHeading((float)angle);
		
		return pose;
	}

	/**
	 * detects parking slots and manage them by initializing new slots, re-characterizing old slots or merge old and detected slots. 
	 */
	private void detectParkingSlot(){
		int newslot;
		// measurements of the front side sensor
		if ( this.detectingSlotFrontSide ) { // back boundary already found
			if ( this.frontSideSensorDistance != Double.POSITIVE_INFINITY ) { // possible front boundary
				if ( this.detectionDelayFrontSide==0 ) { // store first occurence of front boundary
					this.slotBoundariesFrontSide[0] = calcParkingSlotLocation( this.currentLine );
					this.slotBoundariesFrontSide[2] = calcParkingSlotBoundaryDeviation();
					
					} 
				this.detectionDelayFrontSide++; // waiting to add front boundary, until following measures ensure front boundary
				if ( this.detectionDelayFrontSide==DETECTION_DELAY || 
						( this.currentLine.id==4 && this.pose.getX() < 0.5 + OFFSET_PARKSLOT_FRONTSIDE) ) { // front boundary found
					this.detectingSlotFrontSide = false;
					this.detectionDelayFrontSide = 0;
					newslot = this.currentLine.newParkingSlotSpotted(
							parkingSlotIDCounter, slotBoundariesFrontSide[0], slotBoundariesFrontSide[1], OFFSET_PARKSLOT_FRONTSIDE);
					if ( -1== newslot) {
						this.lastSlotChangedID = this.parkingSlotIDCounter;
						this.parkingSlotIDCounter++;
					}
					else {
						this.lastSlotChangedID = newslot;
					}
				}
			}
			else { this.detectionDelayFrontSide = 0; } // still no front boundary
			
		}
		else { // looking for back boundary
			if ( this.frontSideSensorDistance == Double.POSITIVE_INFINITY ) { // possible back boundary
				if ( this.detectionDelayFrontSide==0 ) { // store first occurence of back boundary
					this.slotBoundariesFrontSide[1] = calcParkingSlotLocation( this.currentLine );
					this.slotBoundariesFrontSide[3] = calcParkingSlotBoundaryDeviation();
					} 
				this.detectionDelayFrontSide++; // waiting to add back boundary, until following measures ensure front boundary
				if ( this.detectionDelayFrontSide==DETECTION_DELAY ) { // finalising back boundary, now looking for front boundary
					this.detectingSlotFrontSide = true;
					this.detectionDelayFrontSide = 0;
				}
			}
			else { this.detectionDelayFrontSide = 0; }
		}
		
		// measurements of the back side sensor
		
		if ( this.detectingSlotBackSide ) { // back boundary already found
			if ( this.backSideSensorDistance != Double.POSITIVE_INFINITY ) { // possible front boundary
				if ( this.detectionDelayBackSide==0 ) { // store first occurence of front boundary
					this.slotBoundariesBackSide[0] = calcParkingSlotLocation( this.currentLine );
					this.slotBoundariesBackSide[2] = calcParkingSlotBoundaryDeviation();
					}
				
				this.detectionDelayBackSide++; // waiting to add front boundary, until following measures ensure front boundary
				
				if ( this.detectionDelayBackSide==DETECTION_DELAY || 
						( this.currentLine.id==4 && this.pose.getX() < 0.5 + OFFSET_PARKSLOT_BACKSIDE )) {  // front boundary found
					this.detectingSlotBackSide = false;
					this.detectionDelayBackSide = 0;
					newslot = this.currentLine.newParkingSlotSpotted(
							parkingSlotIDCounter, slotBoundariesBackSide[0], slotBoundariesBackSide[1], OFFSET_PARKSLOT_BACKSIDE );
					if ( newslot == -1 ){
						this.lastSlotChangedID = this.parkingSlotIDCounter;
						this.parkingSlotIDCounter++;
					}
					else {
						this.lastSlotChangedID = newslot;
					}
				}
			}
			else { this.detectionDelayBackSide = 0; }  // still no front boundary
			
		}
		else { // looking for back boundary
			if ( this.backSideSensorDistance == Double.POSITIVE_INFINITY ) { // possible back boundary
				if ( this.detectionDelayBackSide==0 ) { // store first occurence of back boundary
					this.slotBoundariesBackSide[1] = calcParkingSlotLocation( this.currentLine );
					this.slotBoundariesBackSide[3] = calcParkingSlotBoundaryDeviation();
					}
				
				this.detectionDelayBackSide++; // waiting to add back boundary, until following measures ensure front boundary
				
				if ( this.detectionDelayBackSide==DETECTION_DELAY ) { // finalising back boundary, now looking for front boundary
					this.detectingSlotBackSide = true;
					this.detectionDelayBackSide = 0;
				}
			}
			else { this.detectionDelayBackSide = 0; }
		}
		
		
		// special cases on the top line where there is no continuous wall
		
		if ( this.currentLine.id==4 ) {
			this.slotBoundariesBackSide[1]  = Math.min( 1.30 - OFFSET_PARKSLOT_BACKSIDE , this.slotBoundariesBackSide[1]);
			this.slotBoundariesFrontSide[1] = Math.min( 1.30 - OFFSET_PARKSLOT_FRONTSIDE, this.slotBoundariesBackSide[1]);
			
			this.slotBoundariesBackSide[0]  = Math.max( 0.50 - OFFSET_PARKSLOT_BACKSIDE , this.slotBoundariesBackSide[0]);
			this.slotBoundariesFrontSide[0] = Math.max( 0.50 - OFFSET_PARKSLOT_FRONTSIDE, this.slotBoundariesBackSide[0]);
		}
		
		
	}
	
	// functions
/*
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
	
	*/
	private double calcParkingSlotLocation(StraightLine line) {
		double coordinate = 0;
		switch ( line.getDirection() ) {
		/*
			case NORTH:
				coordinate = this.pose.getY() + DISTANCE_ROBOTCOURSE_TO_BORDER * Math.tan( this.pose.getHeading() - Math.PI/2 );
				break;
			case SOUTH:
				coordinate = this.pose.getY() - DISTANCE_ROBOTCOURSE_TO_BORDER * Math.tan( this.pose.getHeading() - Math.PI/2 );
				break;
			case EAST:
				coordinate = this.pose.getX() + DISTANCE_ROBOTCOURSE_TO_BORDER * Math.tan( this.pose.getHeading() );
				break;
			case WEST:
				coordinate = this.pose.getX() - DISTANCE_ROBOTCOURSE_TO_BORDER * Math.tan( this.pose.getHeading() );
				break;
			*/
		case NORTH:
			coordinate = this.pose.getY()
			+ ( DISTANCE_ROBOTCOURSE_TO_BORDER - this.pose.getX() + line.getConstCoordinate() ) * Math.tan( this.pose.getHeading() - Math.PI/2 );
			break;
		case SOUTH:
			coordinate = this.pose.getY()
			- ( DISTANCE_ROBOTCOURSE_TO_BORDER + this.pose.getX() - line.getConstCoordinate() ) * Math.tan( this.pose.getHeading() - Math.PI/2 );
			break;
		case EAST:
			coordinate = this.pose.getX()
			+ ( DISTANCE_ROBOTCOURSE_TO_BORDER + this.pose.getY() - line.getConstCoordinate() ) * Math.tan( this.pose.getHeading() );
			break;
		case WEST:
			coordinate = this.pose.getX()
			- ( DISTANCE_ROBOTCOURSE_TO_BORDER - this.pose.getY() + line.getConstCoordinate() ) * Math.tan( this.pose.getHeading() );
			break;
		}
		return coordinate;
	}
	// 
	private double calcParkingSlotBoundaryDeviation() {
		// using total differential for deviation analysis
		// coordinates given relative to the line
		double deltaX, deltaY, deltaAngle, angle;
		
		angle = currentLine.absAngle(currentLine.getAngle(), pose.getHeading());
		
		if (currentLine.id % 2 == 0) {
			deltaX = this.xDeviation;
			deltaY = Math.tan(angle) * this.yDeviation;
		}
		
		else {
			deltaX = this.yDeviation;
			deltaY = Math.tan(angle) * this.xDeviation;
		}
		deltaAngle = Math.abs( DISTANCE_ROBOTCOURSE_TO_BORDER / Math.pow(Math.cos( angle ), 2) ) * this.angleDeviation;
		
		
		// using sensor's inertia 
		// root mean square
		double x_min = Math.sqrt( ( Math.pow(leftAngleSpeed * LEFT_WHEEL_RADIUS, 2) + Math.pow(rightAngleSpeed * RIGHT_WHEEL_RADIUS, 2) ) / 2) * Math.PI / 180 * 0.06;
		
		return x_min + deltaX + deltaY + deltaAngle;
	}
	
}
