package cam.gun;

import cam.model.BattleInfo;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;

public class CircularGun implements Gun {

	private static CircularGun gun;

	private AdvancedRobot robot;

	public BattleInfo info;
	public double bulletPower;

	private CircularGun(AdvancedRobot robot) {
		this.robot = robot;
	}

	public static CircularGun getGun(AdvancedRobot robot) {
		if (null == gun) {
			gun = new CircularGun(robot);
		}
		return gun;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		// TODO Auto-generated method stub

	}

	@Override
	public void update() {
		// TODO Auto-generated method stub

	}

	@Override
	public long getLastScanTime() {
		return info.lastScanTime;
	}

}
