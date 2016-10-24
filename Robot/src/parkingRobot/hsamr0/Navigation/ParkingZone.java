package parkingRobot.hsamr0.Navigation;

import java.util.LinkedList;
import java.util.List;

import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

public class ParkingZone {
	private List<ParkingSlot> parkingSlots = new LinkedList<ParkingSlot>();
	private Point parkingZoneStart;
	private Point parkingZoneEnd;
	
	public ParkingZone(Point parkingZoneStart, Point parkingZoneEnd){
		this.parkingZoneStart = parkingZoneStart;
	}
	
	/**
	 * Getter for end point
	 * @return startPose
	 */
	public Point getStartPoint(){
		return parkingZoneStart;
	}
	
	/**
	 * Getter for end point
	 * @return endPose
	 */
	public Point getEndPoint(){
		return parkingZoneEnd;
	}
	
	/**
	 * Returns the start point of the parking zone depending on which axis it is parallel to
	 * @return
	 */
	public float get1DStartPoint(){
		if (this.isYParallel())
			return (float)this.parkingZoneStart.getY();
		else
			return (float)this.parkingZoneStart.getX();
	}
	
	/**
	 * Returns the end point of the parking zone depending on which axis it is parallel to
	 * @return
	 */
	public float get1DEndPoint(){
		if (this.isYParallel())
			return (float)this.parkingZoneEnd.getY();
		else
			return (float)this.parkingZoneEnd.getX();
	}
	
	/**
	 * returns true if the parking zone is parallel to the x axis
	 * @return
	 */
	public boolean isXParallel(){
		if(this.parkingZoneStart.getX() == this.parkingZoneEnd.getX())
			return true;
		return false;
	}
	
	/**
	 * returns true if the straight is parallel to the y axis
	 * @return
	 */
	public boolean isYParallel(){
		if(this.parkingZoneStart.getY() == this.parkingZoneEnd.getY())
			return true;
		return false;
	}
}
