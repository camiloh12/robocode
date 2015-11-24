package cam.movement;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.Enumeration;

import cam.model.BattleInfo;
import cam.model.Enemy;
import cam.utils.RoboUtils;
import robocode.AdvancedRobot;
import robocode.Bullet;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

/**
 * Minimum Risk Movement class
 * 
 * This movement is best suited for melee
 *
 */
public class MRMovement implements Movement {

	private static MRMovement movement;

	private AdvancedRobot robot;

	public BattleInfo info;

	private MRMovement(AdvancedRobot robot) {
		this.robot = robot;
	}

	public static MRMovement getMovement(AdvancedRobot robot) {
		if (null == movement) {
			movement = new MRMovement(robot);
		}
		return movement;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {

	}

	@Override
	public void update() {
		double distanceToNextDestination = info.myLocation
				.distance(info.nextDestination);
		double distanceToTarget = info.myLocation
				.distance(info.target.position);

		// search a new destination if I reached this one
		if (distanceToNextDestination < 15) {
			// there should be better formulas then this one but it is basically
			// here to increase 1v1 performance. With more bots addLast
			// will mostly be 1
			double addLast = 1 - Math.rint(Math.pow(Math.random(),
					robot.getOthers()));

			Rectangle2D.Double battlefield = new Rectangle2D.Double(30, 30,
					robot.getBattleFieldWidth() - 60,
					robot.getBattleFieldHeight() - 60);
			Point2D.Double testPoint;

			for (int i = 0; i < 200; i++) {
				/**
				 * Calculate the testPoint somewhere around the current
				 * position. 100 + 200*Math.random() proved to be good if there
				 * are around 10 bots in a 1000x1000 field. But this needs to be
				 * limited to the distanceToTarget*0.8. This way the bot won't
				 * run into the target (should be the closest bot)
				 */
				testPoint = RoboUtils.project(
						info.myLocation,
						2 * Math.PI * Math.random(),
						Math.min(distanceToTarget * 0.8,
								100 + 200 * Math.random()));
				if (battlefield.contains(testPoint)
						&& evaluate(testPoint, addLast) < evaluate(
								info.nextDestination, addLast)) {
					info.nextDestination = testPoint;
				}
			}

			info.lastPosition = info.myLocation;
		} else {
			// just the normal goto stuff
			double angle = RoboUtils.absoluteBearing(info.myLocation,
					info.nextDestination) - robot.getHeadingRadians();
			double direction = 1;

			if (Math.cos(angle) < 0) {
				angle += Math.PI;
				direction = -1;
			}

			robot.setAhead(distanceToNextDestination * direction);
			robot.setTurnRightRadians(angle = Utils.normalRelativeAngle(angle));
			robot.setMaxVelocity(Math.abs(angle) > 1 ? 0 : 8d);
		}
	}

	@Override
	public void updateWaves(Bullet bullet) {

	}

	public double evaluate(Point2D.Double point, double addLast) {
		// this is basically here that the bot uses more space on the
		// battlefield. In melee it is dangerous to stay somewhere too long.
		double eval = addLast * 0.08 / point.distanceSq(info.lastPosition);

		Enumeration<Enemy> enumeration = info.enemies.elements();
		while (enumeration.hasMoreElements()) {
			Enemy enemy = enumeration.nextElement();
			/**
			 * - Math.min(enemy.energy/myEnergy,2) is multiplied because
			 * enemy.energy/myEnergy is an indicator of how dangerous an enemy
			 * is - Math.abs(Math.cos(calAngle(myLocation,point) -
			 * calcAngle(enemy.position,point))) is bigger if the moving
			 * direction isn't good in relation to a certain bot. It would be
			 * more natural to use Math.abs(
			 * Math.cos(calcAngle(point,myLocation) - calcAngle(enemy.poistion,
			 * myLocation))) but this wasn't going to give me good results - 1 /
			 * point.distanceSq(enemy.position) is just the normal anti gravity
			 * thing
			 */
			if (enemy.live) {
				eval += Math.min(enemy.energy / info.myEnergy, 2)
						* (1 + Math.abs(Math.cos(RoboUtils.absoluteBearing(
								point, info.myLocation)
								- RoboUtils.absoluteBearing(point,
										enemy.position))))
						/ point.distanceSq(enemy.position);
			}
		}

		return eval;
	}

}
