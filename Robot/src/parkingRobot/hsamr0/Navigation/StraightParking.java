package parkingRobot.hsamr0.Navigation;

import lejos.robotics.navigation.Pose;

public class StraightParking extends Straight {
	public StraightParking(Pose startPose){
		super(startPose);
	}

	@Override
	public boolean hasParkingSlots() {
		return true;
	}
}
