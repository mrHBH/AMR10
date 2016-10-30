package parkingRobot.hsamr10.Navigation;

import lejos.robotics.navigation.Pose;

public class StraightEmpty extends Straight {
	public StraightEmpty(Pose startPose){
		super(startPose);
	}

	@Override
	public boolean hasParkingZone() {
		return false;
	}
}
