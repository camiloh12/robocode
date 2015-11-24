package cam.movement;

import robocode.Bullet;
import robocode.ScannedRobotEvent;

public interface Movement {

	public void onScannedRobot(ScannedRobotEvent event);

	public void update();

	public void updateWaves(Bullet bullet);
}
