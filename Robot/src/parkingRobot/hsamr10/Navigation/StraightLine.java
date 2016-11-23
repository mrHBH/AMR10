package parkingRobot.hsamr10.Navigation;

import parkingRobot.INavigation;
import parkingRobot.INavigation.ParkingSlot;
import parkingRobot.INavigation.ParkingSlot.ParkingSlotStatus;
import parkingRobot.IPerception;

import lejos.geom.Line;
import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import java.util.LinkedList;
import java.util.List;
//import java.util.Collection;

public class StraightLine {
	private Direction direction;
	private StraightLine followingLine;
	private double startCoordinate;
	private double endCoordinate;
	private double constCoordinate;
	public boolean conditionAngle = false;
	public boolean conditionDistance = false;
	public int id;
	//private LinkedList<ParkingSlot> parkingSlots;
	private List<Double> parkingFrontPoints;
	private List<Double> parkingBackPoints;
	private List<Integer> parkingID;
	
	public enum Direction {
		NORTH,
		EAST,
		SOUTH,
		WEST
	}
	
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
	
	public void setFollowingLine(StraightLine line) {
		this.followingLine = line;
	}
	/*
	public StraightLine getFollowingLine() {
		return this.followingLine;
	}
	*/
	public boolean newParkingSlotSpotted(int ID, double frontBoundaryPosition, double backBoundaryPosition) {
		boolean slotNotInDatabase = true;
		double existingFrontBoundary, existingBackBoundary;
		for (int i=0; i<this.parkingID.size(); i++) {
			existingFrontBoundary = this.parkingFrontPoints.get(i);
			existingBackBoundary  = this.parkingBackPoints.get(i);
			
			// comparing two parking zones with each other. If a front boundary position is in between the other parking slot (or vice versa) they overlap.
			if (this.startCoordinate < this.endCoordinate) {
				if (existingBackBoundary < frontBoundaryPosition && frontBoundaryPosition < existingFrontBoundary &&
					backBoundaryPosition < existingFrontBoundary && existingFrontBoundary < frontBoundaryPosition) {
					
					// setting the boundary position to the location the robot perceives "earlier"
					parkingFrontPoints.set(i, Math.min(existingFrontBoundary, frontBoundaryPosition) );
					parkingBackPoints.set (i, Math.min(existingBackBoundary,  backBoundaryPosition) );
					slotNotInDatabase = false;
				}
			}
			else {
				if (existingBackBoundary > frontBoundaryPosition && frontBoundaryPosition > existingFrontBoundary &&
					backBoundaryPosition > existingFrontBoundary && existingFrontBoundary > frontBoundaryPosition) {
					
					parkingFrontPoints.set(i, Math.max(existingFrontBoundary, frontBoundaryPosition) );
					parkingBackPoints.set (i, Math.max(existingBackBoundary,  backBoundaryPosition) );
					slotNotInDatabase = false;
				}
			}
		}
		
		if (slotNotInDatabase) {
			this.parkingBackPoints.add(backBoundaryPosition);
			this.parkingFrontPoints.add(frontBoundaryPosition);
			this.parkingID.add(ID);
		}
		/*
		ParkingSlot existingParkSlot = null;
		double lowCoordOld = 0;
		double highCoordOld= 0;
		double lowCoordNew = Math.min(frontBoundaryPosition, backBoundaryPosition);
		double highCoordNew= Math.max(frontBoundaryPosition, backBoundaryPosition);
		Point frontParkingZone = new Point(0,0);
		Point backParkingZone  = new Point(0,0);
		for (int i=0; i<this.parkingSlots.size(); i++) {
			existingParkSlot = parkingSlots.get(i);
			switch (this.direction) {
				case EAST:
					lowCoordOld = existingParkSlot.getBackBoundaryPosition().getX();
					highCoordOld= existingParkSlot.getFrontBoundaryPosition().getX();
					break;
				case NORTH:
					lowCoordOld = existingParkSlot.getBackBoundaryPosition().getY();
					highCoordOld= existingParkSlot.getFrontBoundaryPosition().getY();
					break;
				case WEST:
					highCoordOld= existingParkSlot.getBackBoundaryPosition().getX();
					lowCoordOld = existingParkSlot.getFrontBoundaryPosition().getX();
					break;
				case SOUTH:
					highCoordOld= existingParkSlot.getBackBoundaryPosition().getY();
					lowCoordOld = existingParkSlot.getFrontBoundaryPosition().getY();
					break;
			}
			// comparing both parking zones. if one front point is between the other (or vice versa) the parking zones overlap
			if ((	 lowCoordNew < highCoordOld && highCoordOld < highCoordNew) || 
					(lowCoordOld < highCoordNew && highCoordNew < highCoordOld)) {				
				slotNotInDatabase = false;
				// takes the "earlier" locations measured by sensors
				switch (this.direction) {
				case NORTH:
					frontParkingZone.setLocation( this.constCoordinate, Math.min(highCoordNew, highCoordOld) );
					backParkingZone.setLocation(  this.constCoordinate, Math.min(lowCoordNew, lowCoordOld) );
					break;
				case EAST:
					frontParkingZone.setLocation( Math.min(highCoordNew, highCoordOld), this.constCoordinate );
					backParkingZone.setLocation(  Math.min(lowCoordNew, lowCoordOld), this.constCoordinate );
					break;
				case WEST:
					frontParkingZone.setLocation( Math.max(lowCoordNew, lowCoordOld), this.constCoordinate );
					backParkingZone.setLocation(  Math.max(highCoordNew, highCoordOld), this.constCoordinate );
					break;
				case SOUTH:
					frontParkingZone.setLocation( this.constCoordinate, Math.max(lowCoordNew, lowCoordOld) );
					backParkingZone.setLocation(  this.constCoordinate, Math.max(highCoordNew, highCoordOld) );
					break;
			}
				existingParkSlot.setFrontBoundaryPosition(frontParkingZone);
				existingParkSlot.setBackBoundaryPosition(backParkingZone);
				break;
			}

		}
		
		if 		( this.direction==Direction.NORTH || this.direction==Direction.SOUTH) {
			frontParkingZone.setLocation(this.constCoordinate, frontBoundaryPosition);
			backParkingZone.setLocation(  this.constCoordinate, backBoundaryPosition );
		}
		else if ( this.direction==Direction.WEST || this.direction==Direction.EAST) {
			frontParkingZone.setLocation(frontBoundaryPosition, this.constCoordinate);
			backParkingZone.setLocation(  backBoundaryPosition,  this.constCoordinate);
		}
		
		if (slotNotInDatabase){
			ParkingSlotStatus parkingSlotStatus;
			if ( Math.abs( frontBoundaryPosition - backBoundaryPosition ) < 0.45 ) {
				parkingSlotStatus = ParkingSlotStatus.NOT_SUITABLE_FOR_PARKING;
			}
			else {
				parkingSlotStatus = ParkingSlotStatus.SUITABLE_FOR_PARKING;
			}
			
			this.parkingSlots.add(new ParkingSlot(
					ID,
					frontParkingZone,
					backParkingZone,
					parkingSlotStatus,
					0)				
					);
					
		}
		*/
		return slotNotInDatabase;
	}
	
	public Pose poseCorrection(Pose pose, IPerception perception) { //gefährlich. Pose ist eine Klasse
		return null;
	}
	
	public double getStartCoordinate() {
		return this.startCoordinate;
	}
	
	public double getEndCoordinate() {
		return this.endCoordinate;
	}
	
	public double getConstCoordinate() {
		return this.constCoordinate;
	}
	
	public Direction getDirection() {
		return this.direction;
	}
	
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
	
	public StraightLine update( Pose pose ) {
		double angleDifference = Math.min(
				Math.abs( this.getAngle() - pose.getHeading() ),
				Math.abs( this.getAngle() - pose.getHeading() ) + 2*Math.PI ) ;
		double nextAngleDifference = Math.min(
				Math.abs( this.followingLine.getAngle() - pose.getHeading() ),
				Math.abs( this.followingLine.getAngle() - pose.getHeading() ) + 2*Math.PI) ;
		
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
		
		if ( pose.getLocation().distance(x, y) < 0.10 ) {
			this.conditionDistance = true;
		}
		
		if ( this.conditionAngle && this.conditionDistance ) {
		//if (this.conditionAngle) {
			this.conditionAngle = false;
			this.conditionDistance = false;
			return this.followingLine;
		}
		else {
			return this;
		}
	}
	
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
