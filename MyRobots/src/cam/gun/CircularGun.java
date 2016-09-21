package cam.gun;

import java.awt.geom.Point2D;

import cam.model.BattleInfo;
import robocode.AdvancedRobot;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

/**
 * The code for this class is based on the Circular Targeting Tutorial
 */
public class CircularGun implements Gun {

	private static CircularGun gun;

	private AdvancedRobot robot;

	public BattleInfo info;
	public double bulletPower;

	private double oldEnemyHeading;

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
		double bulletPower = 2.0;
		double myX = robot.getX();
		double myY = robot.getY();
		double absoluteBearing = robot.getHeadingRadians() + event.getBearingRadians();
		double enemyX = robot.getX() + event.getDistance() * Math.sin(absoluteBearing);
		double enemyY = robot.getY() + event.getDistance() * Math.cos(absoluteBearing);
		double enemyHeading = event.getHeadingRadians();
		double enemyHeadingChange = enemyHeading - oldEnemyHeading;
		double enemyVelocity = event.getVelocity();
		oldEnemyHeading = enemyHeading;

		double deltaTime = 0;
		double battleFieldHeight = robot.getBattleFieldHeight(), battleFieldWidth = robot.getBattleFieldWidth();
		double predictedX = enemyX, predictedY = enemyY;
		while ((++deltaTime) * (20.0 - 3.0 * bulletPower) < Point2D.Double.distance(myX, myY, predictedX, predictedY)) {
			predictedX += Math.sin(enemyHeading) * enemyVelocity;
			predictedY += Math.cos(enemyHeading) * enemyVelocity;
			enemyHeading += enemyHeadingChange;
			if (predictedX < 18.0 || predictedY < 18.0 || predictedX > battleFieldWidth - 18.0
					|| predictedY > battleFieldHeight - 18.0) {

				predictedX = Math.min(Math.max(18.0, predictedX), battleFieldWidth - 18.0);
				predictedY = Math.min(Math.max(18.0, predictedY), battleFieldHeight - 18.0);
				break;
			}
		}
		double theta = Utils.normalAbsoluteAngle(Math.atan2(predictedX - robot.getX(), predictedY - robot.getY()));

		robot.setTurnRadarRightRadians(Utils.normalRelativeAngle(absoluteBearing - robot.getRadarHeadingRadians()));
		robot.setTurnGunRightRadians(Utils.normalRelativeAngle(theta - robot.getGunHeadingRadians()));
		robot.fire(bulletPower);

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
