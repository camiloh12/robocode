package cam.gun;

import java.awt.geom.Point2D;

import cam.utils.RoboUtils;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

/**
 * Guess Factor Targeting Gun
 * 
 * This gun is best suited for 1v1
 *
 */
public class GFTGun implements Gun {

	private static final double BULLET_POWER = 1.8;

	private static GFTGun gun;

	private AdvancedRobot robot;
	private double lateralDirection;
	private double lastEnemyVelocity;
	private long lastScanTime;

	private GFTGun(AdvancedRobot robot) {
		this.robot = robot;
		lateralDirection = 1;
		lastEnemyVelocity = 0;
		lastScanTime = 0;
	}

	public static GFTGun getGun(AdvancedRobot robot) {
		if (null == gun || robot.getTime() == 0) {
			gun = new GFTGun(robot);
		}
		return gun;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		lastScanTime = robot.getTime();
		double enemyAbsoluteBearing = robot.getHeadingRadians()
				+ event.getBearingRadians();
		double enemyDistance = event.getDistance();
		double enemyVelocity = event.getVelocity();

		if (enemyVelocity != 0) {
			lateralDirection = RoboUtils
					.sign(enemyVelocity
							* Math.sin(event.getHeadingRadians()
									- enemyAbsoluteBearing));
		}
		GFTWave wave = new GFTWave(robot);
		wave.gunLocation = new Point2D.Double(robot.getX(), robot.getY());
		GFTWave.targetLocation = RoboUtils.project(wave.gunLocation,
				enemyAbsoluteBearing, enemyDistance);
		wave.lateralDirection = lateralDirection;
		wave.bulletPower = Math.min(BULLET_POWER, Math.min(robot.getEnergy()/16, event.getEnergy()/2));
		wave.setSegmentations(enemyDistance, enemyVelocity, lastEnemyVelocity);
		lastEnemyVelocity = enemyVelocity;
		wave.bearing = enemyAbsoluteBearing;
		robot.setTurnGunRightRadians(Utils
				.normalRelativeAngle(enemyAbsoluteBearing
						- robot.getGunHeadingRadians()
						+ wave.mostVisitedBearingOffset()));
		if (robot.getEnergy() > wave.bulletPower) {
			robot.setFire(wave.bulletPower);
			robot.addCustomEvent(wave);
		}
		robot.setTurnRadarRightRadians(Utils
				.normalRelativeAngle(enemyAbsoluteBearing
						- robot.getRadarHeadingRadians()) * 2);
	}

	@Override
	public void update() {
		// basic mini-radar code
		if (Math.abs(robot.getRadarTurnRemaining()) < 0.000001
				&& robot.getOthers() > 0) {
			robot.setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
		}
	}

	@Override
	public long getLastScanTime() {
		return lastScanTime;
	}

}
