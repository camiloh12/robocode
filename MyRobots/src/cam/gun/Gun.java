package cam.gun;

import robocode.ScannedRobotEvent;

public interface Gun {

	public void onScannedRobot(ScannedRobotEvent event);

	public void update();

	public long getLastScanTime();
}
