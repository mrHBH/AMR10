package parkingRobot.hsamr10.Navigation;

import lejos.geom.Point;
import lejos.robotics.navigation.Pose;

import java.util.LinkedList;
import java.util.List;

public class StraightParking extends Straight {
	private ParkingZone parkingZone;

	
	public StraightParking(Pose startPose, Point parkingZoneStart, Point parkingZoneEnd){
		super(startPose);
		this.parkingZone = new ParkingZone(parkingZoneStart, parkingZoneEnd);
	}

	@Override
	public boolean hasParkingZone() {
		return true;
	}
	
	public ParkingZone getParkingZone(){
		return this.parkingZone;
	}
}