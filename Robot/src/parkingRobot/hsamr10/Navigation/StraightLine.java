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
//import java.util.Collection;

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
	static private final double OFFSET_PARKSLOT_BACKSIDE  = 0.100;
	static private final double OFFSET_PARKSLOT_FRONTSIDE = 0.075;
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
		this.parkingFrontPoints = new LinkedList<Double>();
		this.parkingBackPoints  = new LinkedList<Double>();
		this.parkingID		 	= new LinkedList<Integer>();
		//this.parkingSlots = new LinkedList<ParkingSlot>();
	}
	/**
	 * sets the following line. Method to be called once.
	 * @param line  line given as an four-dimensional array with starting coordinates x1, y1 and target coordinates x2, y2
	 */
	public void setFollowingLine(StraightLine line) {
		this.followingLine = line;
	}
	/*
	public StraightLine getFollowingLine() {
		return this.followingLine;
	}
	*/
	/**
	 * navigation module spots a parking slot during it's scout mode. method evaluates it's parameters, and existence in a current database
	 * 
	 * @param ID  id to be given, if the slot is new
	 * @param frontBoundaryPosition  position of parking slot front boundary, robot passes it second after the back boundary
	 * @param backBoundaryPosition  position of parking slot back boundary, robot passes it first then the front boundary
	 * @return  true, if the parking slot added as a new one in the database (and therefore the ID counter has to be incremented)
	 */
	public boolean newParkingSlotSpotted(int ID, double frontBoundaryPosition, double backBoundaryPosition) {
		if ( this.id==5 ) { return false; } // there is no parking slot, robot is turning to a street
		
		boolean slotNotInDatabase = true;
		
		// maybe this condition is not necessary...
		if ( (this.startCoordinate < this.endCoordinate && backBoundaryPosition > frontBoundaryPosition) || 
			 (this.startCoordinate > this.endCoordinate && backBoundaryPosition < frontBoundaryPosition)	) {
			return false; // ParkingSlot cannot exist, this behaviour might appear at the very edge of a line
		}
		
		if (this.startCoordinate < this.endCoordinate) {
			 backBoundaryPosition  -= OFFSET_PARKSLOT_BACKSIDE;
			 frontBoundaryPosition += OFFSET_PARKSLOT_FRONTSIDE;
		}
		else {
			 backBoundaryPosition  += OFFSET_PARKSLOT_BACKSIDE;
			 frontBoundaryPosition -= OFFSET_PARKSLOT_FRONTSIDE;
		}
		
		
		double existingFrontBoundary, existingBackBoundary;
		for (int i=0; i<this.parkingID.size(); i++) {
			existingFrontBoundary = this.parkingFrontPoints.get(i);
			existingBackBoundary  = this.parkingBackPoints.get(i);
			
			// comparing two parking zones with each other. If a front boundary position is in between the other parking slot (OR vice versa) they overlap.
			if (this.startCoordinate < this.endCoordinate) {
				if ((existingBackBoundary < frontBoundaryPosition && frontBoundaryPosition < existingFrontBoundary) ||
					(backBoundaryPosition < existingFrontBoundary && existingFrontBoundary < frontBoundaryPosition)) {
					
					// setting the boundary position to the location the robot perceives "earlier"
					parkingFrontPoints.set(i, Math.min(existingFrontBoundary, frontBoundaryPosition) );
					parkingBackPoints.set (i, Math.min(existingBackBoundary,  backBoundaryPosition) );
					slotNotInDatabase = false;
					ID = i;
				}
			}
			else {
				if ((existingBackBoundary > frontBoundaryPosition && frontBoundaryPosition > existingFrontBoundary) ||
					(backBoundaryPosition > existingFrontBoundary && existingFrontBoundary > frontBoundaryPosition)) {
					
					parkingFrontPoints.set(i, Math.max(existingFrontBoundary, frontBoundaryPosition) );
					parkingBackPoints.set (i, Math.max(existingBackBoundary,  backBoundaryPosition) );
					slotNotInDatabase = false;
					ID = i;
				}
			}
		}
		String comment = new String();
		if (slotNotInDatabase) {
			this.parkingBackPoints.add(backBoundaryPosition);
			this.parkingFrontPoints.add(frontBoundaryPosition);
			this.parkingID.add(ID);
			
			comment = "new Slot: ";
		}
//
		else {
			backBoundaryPosition = this.parkingBackPoints.get( ID );
			frontBoundaryPosition = this.parkingFrontPoints.get( ID );
			
			comment = "old Slot: ";
		}
		this.monitor.writeNavigationComment(comment + backBoundaryPosition + " " + frontBoundaryPosition + " " + this.id);

		return slotNotInDatabase;
	}
	/**
	 * further refining of the x, y, coordinates and the robot's angle
	 * 
	 * @param pose  current robot pose
	 * @param perception  module, to use advanced localisation measurements
	 */
	public void poseCorrection(Pose pose, IPerception perception) {
		 
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
	 * @return  returns this line's angle relative to the positive x-axis according to its angle
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
		double angleDifference = Math.min(
				Math.abs( this.getAngle() - pose.getHeading() ),
				Math.abs( this.getAngle() - pose.getHeading() + 2*Math.PI ) ) ;
		double nextAngleDifference = Math.min(
				Math.abs( this.followingLine.getAngle() - pose.getHeading() ),
				Math.abs( this.followingLine.getAngle() - pose.getHeading() + 2*Math.PI ) ) ;
		
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
			switch( this.followingLine.direction ) {
			case EAST:
				x = pose.getX();
				y = this.followingLine.constCoordinate;
				break;
			case WEST:
				x = pose.getX();
				y = this.followingLine.constCoordinate;
				break;
			case NORTH:
				x = this.followingLine.constCoordinate;
				y = pose.getY();
				break;
			case SOUTH:
				x = this.followingLine.constCoordinate;
				y = pose.getY();
				break;
			}
			pose.setLocation( (float)x, (float)y);			
			
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
	 * @return  returns a list of parking slot this line is containing
	 */
	public List<ParkingSlot> getParkingSlots() {		
		List<ParkingSlot> parkingSlots = new LinkedList<ParkingSlot>();
		Point frontBoundaryPosition = null;
		Point backBoundaryPosition  = null;
		ParkingSlotStatus parkingSlotStatus = null;
		
		for (int i=0; i<this.parkingID.size(); i++) {
			
			if 		( this.direction==Direction.NORTH || this.direction==Direction.SOUTH) {
				frontBoundaryPosition = new Point(
						(float)this.constCoordinate,
						(float)this.parkingFrontPoints.get(i).doubleValue());
				
				backBoundaryPosition = new Point(
						(float)this.constCoordinate,
						(float)this.parkingBackPoints.get(i).doubleValue());
			}
			else if ( this.direction==Direction.WEST || this.direction==Direction.EAST) {
				frontBoundaryPosition = new Point(
						(float)this.parkingFrontPoints.get(i).doubleValue(),
						(float)this.constCoordinate);
				
				backBoundaryPosition = new Point(
						(float)this.parkingBackPoints.get(i).doubleValue(),
						(float)this.constCoordinate);
			}
			
			if ( Math.abs(this.parkingFrontPoints.get(i)-this.parkingBackPoints.get(i)) > 0.45 ) {
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
	
}
