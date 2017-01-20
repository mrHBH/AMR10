package parkingRobot.hsamr10.Navigation;

import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;
import parkingRobot.IMonitor;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import java.util.LinkedList;
import java.util.List;

/**
 * class to handle line specific calculations for each part of the map
 * 
 * @author Tung Le
 *
 */


public class StraightLine {
	private Direction direction;
	private StraightLine followingLine;
	private double startCoordinate;
	private double endCoordinate;
	private double constCoordinate;
	private boolean conditionAngle = false;
	private boolean conditionDistance = false;
	public int id;
	//private LinkedList<ParkingSlot> parkingSlots;
	private List<Double> parkingFrontPoints;
	private List<Double> parkingBackPoints;
	private List<Integer> parkingID;
	private List<Integer> parkingCount;
	private List<Double> parkingMeasurementQualityFront;
	private List<Double> parkingMeasurementQualityBack;


	/**
	 * used for monitoring purposes
	 */
	public IMonitor monitor = null;

	
	/**
	 * defines the direction a robot can follow a line
	 * 
	 * @author Tung Le
	 *
	 */
	public enum Direction {
		NORTH,
		EAST,
		SOUTH,
		WEST
	}
	
	private static final double DISTANCE_LIGHTSENSORS_ROBOTCENTRE = 0.06;
	
	
	/**
	 * Class constructor. Defines the lines direction according to start and target points
	 * 
	 * @param line  line given as an four-dimensional array with starting coordinates x1, y1 and target coordinates x2, y2
	 */
	public StraightLine(Line line) {		
		if ( line.getX1() == line.getX2() ) {
			this.startCoordinate = line.getY1()/100;
			this.endCoordinate   = line.getY2()/100;
			this.constCoordinate = line.getX1()/100;
			if 		( startCoordinate < endCoordinate )  {this.direction = Direction.NORTH; }
			else if ( startCoordinate > endCoordinate )  {this.direction = Direction.SOUTH; }
		}
		else if ( line.getY1() == line.getY2() ) {
			this.startCoordinate = line.getX1()/100;
			this.endCoordinate   = line.getX2()/100;
			this.constCoordinate = line.getY1()/100;
			if 		( startCoordinate < endCoordinate )  {this.direction = Direction.EAST; }
			else if ( startCoordinate > endCoordinate )  {this.direction = Direction.WEST; }
		}
		this.parkingFrontPoints			= new LinkedList<Double>();
		this.parkingBackPoints			= new LinkedList<Double>();
		this.parkingID		 			= new LinkedList<Integer>();
		this.parkingCount				= new LinkedList<Integer>();
		this.parkingMeasurementQualityFront	= new LinkedList<Double>();
		this.parkingMeasurementQualityBack	= new LinkedList<Double>();
		
		
	}
	/**
	 * sets the following line. Method to be called once, unless it's a dynamic map.
	 * @param line  line given as an four-dimensional array with starting coordinates x1, y1 and target coordinates x2, y2
	 */
	public void setFollowingLine(StraightLine line) {
		this.followingLine = line;
	}


	/**
	 * navigation module spots a parking slot during it's scout mode. method evaluates it's parameters, and existence in a current database
	 * 
	 * @param ID  id to be given, if the slot is new
	 * @param frontBoundaryPosition  position of parking slot front boundary, robot passes it second after the back boundary
	 * @param backBoundaryPosition  position of parking slot back boundary, robot passes it first then the front boundary
	 * @param offset  sensor's location relative to the robot's centre (wheel axis)
	 * @param frontDeviation 
	 * @param backDeviation 
	 * @return  -1, if the parking slot added as a new one in the database (and therefore the ID counter has to be incremented),
	 * otherwise false.
	 */
	public int newParkingSlotSpotted(int ID,
			double frontBoundaryPosition, double backBoundaryPosition,
			double offset,
			double frontDeviation, double backDeviation) {
		if ( this.id==5 || this.id==3 ) { return -1; } // there is no parking slot, robot is turning from / to a street		

		
		if ( (this.startCoordinate < this.endCoordinate && backBoundaryPosition > frontBoundaryPosition) || 
			 (this.startCoordinate > this.endCoordinate && backBoundaryPosition < frontBoundaryPosition)	) {
			return -1; // ParkingSlot cannot exist, this behaviour might appear at the very edge of a line
		}
		
		if ( Math.abs( frontBoundaryPosition - backBoundaryPosition ) < 0.05 ) { // always disregard parking slots smaller than the length of 50 mm
			return -1;
		}
		
		boolean slotNotInDatabase = true;
		
		if (this.startCoordinate < this.endCoordinate) {
			 backBoundaryPosition  += offset;
			 frontBoundaryPosition += offset;
		}
		else {
			 backBoundaryPosition  -= offset;
			 frontBoundaryPosition -= offset;
		}
		
		
		int index = 0;

		double existingFrontBoundary, existingBackBoundary, existingFrontDeviation, existingBackDeviation, newFrontBoundary, newBackBoundary;
		for (int i=0; i<this.parkingID.size(); i++) {
			existingFrontBoundary = this.parkingFrontPoints.get(i);
			existingBackBoundary  = this.parkingBackPoints.get(i);
			existingFrontDeviation = this.parkingMeasurementQualityFront.get(i);
			existingBackDeviation  = this.parkingMeasurementQualityBack.get(i);
			
			// comparing two parking zones with each other. If a front boundary position is in between the other parking slot (OR vice versa) they overlap.
			if (this.startCoordinate < this.endCoordinate) {
				if ((existingBackBoundary < frontBoundaryPosition && frontBoundaryPosition < existingFrontBoundary) ||
					(backBoundaryPosition < existingFrontBoundary && existingFrontBoundary < frontBoundaryPosition)) {
					
					// setting the boundary position to the location the robot perceives with higher precision
					//parkingFrontPoints.set(i, Math.min(existingFrontBoundary, frontBoundaryPosition) );
					//parkingBackPoints.set (i, Math.min(existingBackBoundary,  backBoundaryPosition) );
					newFrontBoundary = (existingFrontBoundary * frontDeviation + frontBoundaryPosition * existingFrontDeviation) /
										(frontDeviation + existingFrontDeviation);
					newBackBoundary = (existingBackBoundary * backDeviation + backBoundaryPosition * existingBackDeviation) /
										(backDeviation + existingBackDeviation);
					
					parkingFrontPoints.set(i, newFrontBoundary);
					parkingBackPoints.set (i, newBackBoundary );
					
					parkingMeasurementQualityFront.set(i, Math.min(frontDeviation, existingFrontDeviation));
					parkingMeasurementQualityBack.set(i, Math.min(backDeviation, existingBackDeviation));
										
					slotNotInDatabase = false;
					index = i;
				}
			}
			else {
				if ((existingBackBoundary > frontBoundaryPosition && frontBoundaryPosition > existingFrontBoundary) ||
					(backBoundaryPosition > existingFrontBoundary && existingFrontBoundary > frontBoundaryPosition)) {
					
					//parkingFrontPoints.set(i, Math.max(existingFrontBoundary, frontBoundaryPosition) );
					//parkingBackPoints.set (i, Math.max(existingBackBoundary,  backBoundaryPosition) );
					
					newFrontBoundary = (existingFrontBoundary * frontDeviation + frontBoundaryPosition * existingFrontDeviation) /
										(frontDeviation + existingFrontDeviation);
					newBackBoundary = (existingBackBoundary * backDeviation + backBoundaryPosition * existingBackDeviation) /
										(backDeviation + existingBackDeviation);
					
					parkingFrontPoints.set(i, newFrontBoundary);
					parkingBackPoints.set (i, newBackBoundary );
					
					parkingMeasurementQualityFront.set(i, Math.min(frontDeviation, existingFrontDeviation));
					parkingMeasurementQualityBack.set(i, Math.min(backDeviation, existingBackDeviation));
									
					
					slotNotInDatabase = false;
					index = i;
				}
			}
		}
		
		String comment = new String();
		if (slotNotInDatabase) {
			this.parkingBackPoints.add(backBoundaryPosition);
			this.parkingFrontPoints.add(frontBoundaryPosition);
			this.parkingID.add(ID);
			this.parkingCount.add(1);
			this.parkingMeasurementQualityFront.add( frontDeviation );
			this.parkingMeasurementQualityBack.add(   backDeviation );
			
			comment = "1.M: ";
			
		}
//
		else {
			index = 0;
			backBoundaryPosition = this.parkingBackPoints.get( index );
			frontBoundaryPosition = this.parkingFrontPoints.get( index );
			parkingCount.set(index, parkingCount.get(index)+1);
			backDeviation = this.parkingMeasurementQualityBack.get( index );
			frontDeviation = this.parkingMeasurementQualityFront.get( index );
			
			comment = parkingCount.get(index) + ".M: ";
		}
		comment = comment + "ID: " +this.id +"  Back: " + backBoundaryPosition + "  Front: " + frontBoundaryPosition;
		comment = comment  + "  deltab:" + backDeviation + "  deltaf:" + frontDeviation;
		this.monitor.writeNavigationComment(comment);
		
		if (slotNotInDatabase) {
			return -1;
		}
		else {
			return ID;
		}
		
	}
	/**
	 * 
	 * further refining of the x, y, coordinates and the robot's angle
	 * 
	 * @param pose current robot pose
	 * @param lineSensorRight
	 * @param lineSensorLeft
	 * @param frontSensorDistance
	 * @param frontSideSensorDistance
	 * @param backSensorDistance
	 * @param backSideSensorDistance
	 */
	public void poseCorrection(
			Pose pose,
			int lineSensorRight,
			int lineSensorLeft,
			double frontSensorDistance,
			double frontSideSensorDistance,
			double backSensorDistance,
			double backSideSensorDistance
			) {
		
	}
	
	
	/**
	 * starting coordinate of this line
	 * 
	 * @return  the relevant coordinate one-dimensional. In case of East and West, the x-component, in case of North and South, the y-component
	 */
	public double getStartCoordinate() {
		return this.startCoordinate;
	}
	/**
	 * target coordinate of this line
	 * 
	 * @return  the relevant coordinate one-dimensional. In case of East and West, the x-component, in case of North and South, the y-component
	 */
	public double getEndCoordinate() {
		return this.endCoordinate;
	}
	/**
	 * constant component of this line
	 * 
	 * @return  the relevant coordinate one-dimensional. In case of North and South, the x-component, in case of East and West, the y-component
	 */
	public double getConstCoordinate() {
		return this.constCoordinate;
	}
	/**
	 * 
	 * @return  returns this line's direction.
	 */
	public Direction getDirection() {
		return this.direction;
	}
	/**
	 * 
	 * @return  returns this line's angle relative to the positive x-axis according to its direction
	 */
	public double getAngle() {
		double angle;
		switch (this.direction) {
			case EAST:
				angle = 0 * Math.PI/2;
				break;
			case NORTH:
				angle = 1 * Math.PI/2;
				break;
			case WEST:
				angle = 2 * Math.PI/2;
				break;
			case SOUTH:
				angle = 3 * Math.PI/2;
				break;
			default:
				angle = 0;
		}
		return angle;
	}	
	/**
	 * calculates, whether the robot is switching lines. resets coordinates due to known turning points
	 * @param pose  current robot pose
	 * @return  this line or the following line in case the robot is taking a turn
	 */
	public StraightLine update( Pose pose ) {
		double angleDifference		= absAngle( this.getAngle(), pose.getHeading() );
		double nextAngleDifference	= absAngle( this.followingLine.getAngle(), pose.getHeading() );
		
		if ( angleDifference > nextAngleDifference ) {
			this.conditionAngle = true;
		}
		
		// x, y sind die Zielkoordinaten
		double x, y;
		switch (this.direction) {
			case EAST:
				x = this.endCoordinate;
				y = this.constCoordinate;
				break;
			case WEST:
				x = this.endCoordinate;
				y = this.constCoordinate;
				break;
			case NORTH:
				x = this.constCoordinate;
				y = this.endCoordinate;
				break;
			case SOUTH:
				x = this.constCoordinate;
				y = this.endCoordinate;
				break;
			default:
				x = y = 0;
		}
		
		if ( pose.getLocation().distance(x, y) < 0.20 ) {
			this.conditionDistance = true;
		}
		
		if ( this.conditionAngle && this.conditionDistance ) {
		//if (this.conditionAngle) {
			// Reset one axis (currently hard, maybe implement with filter)
			
			
			switch( this.direction ) {
			case NORTH:
				x = pose.getX();
				y = ( this.followingLine.constCoordinate - DISTANCE_LIGHTSENSORS_ROBOTCENTRE + pose.getY() )/ 2;
				break;
			case SOUTH:
				x = pose.getX();
				y = ( this.followingLine.constCoordinate + DISTANCE_LIGHTSENSORS_ROBOTCENTRE + pose.getY() )/ 2;
				break;
			case EAST:
				x = ( this.followingLine.constCoordinate - DISTANCE_LIGHTSENSORS_ROBOTCENTRE + pose.getX() )/ 2;
				y = pose.getY();
				break;
			case WEST:
				x = ( this.followingLine.constCoordinate + DISTANCE_LIGHTSENSORS_ROBOTCENTRE + pose.getX() )/ 2;
				y = pose.getY();
				break;
			}
			//pose.setLocation( (float)x, (float)y);			
			
			this.conditionAngle = false;
			this.conditionDistance = false;
			
			
			return this.followingLine;
		}
		else {
			return this;
		}
	}
	/**
	 * 
	 * @return  returns a list of parking slots this line is containing
	 */
	public List<ParkingSlot> getParkingSlots() {		
		List<ParkingSlot> parkingSlots = new LinkedList<ParkingSlot>();
		Point frontBoundaryPosition = null;
		Point backBoundaryPosition  = null;
		ParkingSlotStatus parkingSlotStatus = null;
		
		for (int i=0; i<this.parkingID.size(); i++) {
			
			if 		( this.direction==Direction.NORTH || this.direction==Direction.SOUTH) {
				frontBoundaryPosition = new Point(
						(float)this.constCoordinate * 100,
						(float)this.parkingFrontPoints.get(i).doubleValue() * 100);
				
				backBoundaryPosition = new Point(
						(float)this.constCoordinate * 100,
						(float)this.parkingBackPoints.get(i).doubleValue() * 100);
			}
			else if ( this.direction==Direction.WEST || this.direction==Direction.EAST) {
				frontBoundaryPosition = new Point(
						(float)this.parkingFrontPoints.get(i).doubleValue() * 100,
						(float)this.constCoordinate * 100) ;
				
				backBoundaryPosition = new Point(
						(float)this.parkingBackPoints.get(i).doubleValue() * 100,
						(float)this.constCoordinate * 100);
			}
			
			if ( Math.abs(this.parkingFrontPoints.get(i)-this.parkingBackPoints.get(i)) > 0.40 ) {
				parkingSlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
			}
			else {
				parkingSlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
			}
			
			parkingSlots.add( new ParkingSlot(
					this.parkingID.get(i),
					backBoundaryPosition,
					frontBoundaryPosition,
					parkingSlotStatus,
					0
					));
		}
		return parkingSlots;
	}
	
	public double absAngle (double angle1, double angle2) {
		return Math.abs( Math.min ( Math.abs( angle1-angle2 ), 2*Math.PI-Math.abs( angle1-angle2 ) ) ) % (2*Math.PI);
	}
	
}