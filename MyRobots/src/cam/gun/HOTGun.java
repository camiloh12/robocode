package cam.gun;

import java.awt.geom.Point2D;

import cam.model.BattleInfo;
import cam.model.Enemy;
import cam.utils.RoboUtils;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

/**
 * Head-On Targeting Gun
 * 
 * This gun is best suited for melee
 *
 */
public class HOTGun implements Gun {

	private static HOTGun gun;

	private AdvancedRobot robot;

	public BattleInfo info;

	private HOTGun(AdvancedRobot robot) {
		this.robot = robot;
	}

	public static HOTGun getGun(AdvancedRobot robot) {
		if (null == gun) {
			gun = new HOTGun(robot);
		}
		return gun;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		info.myLocation = new Point2D.Double(robot.getX(), robot.getY());
		info.lastScanTime = robot.getTime();
		info.myEnergy = robot.getEnergy();

		Enemy enemy = info.enemies.get(event.getName());

		if (null == enemy) {
			enemy = new Enemy();
			info.enemies.put(event.getName(), enemy);
		}

		enemy.energy = event.getEnergy();
		enemy.live = true;
		enemy.position = RoboUtils.project(info.myLocation, robot.getHeadingRadians() + event.getBearingRadians(),
				event.getDistance());

		// normal target selection: the one closer to you is the most dangerous
		// so attack him
		if (!info.target.live || event.getDistance() < info.myLocation.distance(info.target.position)) {
			info.target = enemy;
		}

		// locks the radar if there is only one opponent left
		if (robot.getOthers() == 1) {
			robot.setTurnRadarLeftRadians(robot.getRadarTurnRemainingRadians());
		} else {
			robot.setTurnGunRightRadians(Utils.normalRelativeAngle(
					RoboUtils.absoluteBearing(info.myLocation, info.target.position) - robot.getGunHeadingRadians()));
		}
	}

	@Override
	public void update() {
		// HeadOnTargeting
		double distanceToTarget = info.myLocation.distance(info.target.position);
		if (robot.getGunTurnRemainingRadians() == 0 && info.myEnergy > 1) {
			robot.setFire(Math.min(Math.min(info.myEnergy / 6d, 1300d / distanceToTarget), info.target.energy / 3d));
		}
	}

	@Override
	public long getLastScanTime() {
		return info.lastScanTime;
	}

}
