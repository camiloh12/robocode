package cam.gun;

import cam.model.BattleInfo;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;

public class LinearGun implements Gun {

	private static LinearGun gun;

	private AdvancedRobot robot;

	public BattleInfo info;

	private LinearGun(AdvancedRobot robot) {
		this.robot = robot;
	}

	public static LinearGun getGun(AdvancedRobot robot) {
		if (null == gun) {
			gun = new LinearGun(robot);
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
