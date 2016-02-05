package cam.movement;

import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.List;

import cam.model.BattleInfo;
import cam.model.Enemy;
import cam.model.EnemyWave;
import cam.utils.RoboUtils;
import robocode.AdvancedRobot;
import robocode.Bullet;
import robocode.ScannedRobotEvent;
import robocode.util.Utils;

public class CombinedMovement implements Movement {

	public BattleInfo info;

	private static final double WALL_MARGIN = 18;
	private static final double WALL_STICK = 160;
	private static final int BINS = 47;
	/**
	 * Math.PI/2 would be perpendicular movement, less will keeps us moving away
	 * slightly
	 */
	private static final double LESS_THAN_HALF_PI = 1.25;

	private static double oppEnergy = 100.0;
	private static double surfStats[] = new double[BINS];

	private AdvancedRobot robot;
	// Wall Smoothing
	private Rectangle2D.Double fieldRectangle;
	private Point2D.Double myLocation;
	private Point2D.Double enemyLocation;

	private List<EnemyWave> enemyWaves = new ArrayList<>();
	private List<Integer> surfDirections = new ArrayList<>();
	private List<Double> surfAbsBearings = new ArrayList<>();

	public CombinedMovement(AdvancedRobot robot) {
		this.robot = robot;
		fieldRectangle = new Rectangle2D.Double(WALL_MARGIN, WALL_MARGIN, robot.getBattleFieldWidth() - WALL_MARGIN * 2,
				robot.getBattleFieldHeight() - WALL_MARGIN * 2);
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		// TODO WSMovement
		if (robot.getOthers() <= 1) {
			myLocation = new Point2D.Double(robot.getX(), robot.getY());

			double lateralVelocity = robot.getVelocity() * Math.sin(event.getBearingRadians());
			double absBearing = event.getBearingRadians() + robot.getHeadingRadians();

			robot.setTurnRadarRightRadians(Utils.normalRelativeAngle(absBearing - robot.getRadarHeadingRadians() * 2));

			surfDirections.add(0, new Integer((lateralVelocity >= 0) ? 1 : -1));
			surfAbsBearings.add(0, new Double(absBearing + Math.PI));

			double bulletPower = oppEnergy - event.getEnergy();
			if (bulletPower < 3.01 && bulletPower > 0.09 && surfDirections.size() > 2) {
				EnemyWave enemyWave = new EnemyWave();
				enemyWave.fireTime = robot.getTime() + 1;
				enemyWave.bulletVelocity = RoboUtils.bulletVelocity(bulletPower);
				enemyWave.distanceTraveled = RoboUtils.bulletVelocity(bulletPower);
				;
				enemyWave.direction = surfDirections.get(2).intValue();
				enemyWave.directAngle = surfAbsBearings.get(2).doubleValue();
				enemyWave.fireLocation = (Point2D.Double) enemyLocation.clone();
				enemyWaves.add(enemyWave);
			}

			oppEnergy = event.getEnergy();

			// update after EnemyWave detection, because that needs the previous
			// enemy location as the source of the wave
			enemyLocation = RoboUtils.project(myLocation, absBearing, event.getDistance());

			updateWaves();
			doSurfing();
		}

	}

	@Override
	public void update() {

		if (robot.getOthers() > 1) {
			// TODO MRMovement
			double distanceToNextDestination = info.myLocation.distance(info.nextDestination);
			double distanceToTarget = info.myLocation.distance(info.target.position);

			// search a new destination if I reached this one
			if (distanceToNextDestination < 15) {
				// there should be better formulas then this one but it is
				// basically
				// here to increase 1v1 performance. With more bots addLast
				// will mostly be 1
				double addLast = 1 - Math.rint(Math.pow(Math.random(), robot.getOthers()));

				Rectangle2D.Double battlefield = new Rectangle2D.Double(30, 30, robot.getBattleFieldWidth() - 60,
						robot.getBattleFieldHeight() - 60);
				Point2D.Double testPoint;

				for (int i = 0; i < 200; i++) {
					/**
					 * Calculate the testPoint somewhere around the current
					 * position. 100 + 200*Math.random() proved to be good if
					 * there are around 10 bots in a 1000x1000 field. But this
					 * needs to be limited to the distanceToTarget*0.8. This way
					 * the bot won't run into the target (should be the closest
					 * bot)
					 */
					testPoint = RoboUtils.project(info.myLocation, 2 * Math.PI * Math.random(),
							Math.min(distanceToTarget * 0.8, 100 + 200 * Math.random()));
					if (battlefield.contains(testPoint)
							&& evaluate(testPoint, addLast) < evaluate(info.nextDestination, addLast)) {
						info.nextDestination = testPoint;
					}
				}

				info.lastPosition = info.myLocation;
			} else {
				// just the normal goto stuff
				double angle = RoboUtils.absoluteBearing(info.myLocation, info.nextDestination)
						- robot.getHeadingRadians();
				double direction = 1;

				if (Math.cos(angle) < 0) {
					angle += Math.PI;
					direction = -1;
				}

				robot.setAhead(distanceToNextDestination * direction);
				robot.setTurnRightRadians(angle = Utils.normalRelativeAngle(angle));
				robot.setMaxVelocity(Math.abs(angle) > 1 ? 0 : 8d);
			}
		} else {
			// TODO WSMovement
			myLocation = new Point2D.Double(robot.getX(), robot.getY());
			updateWaves();
			doSurfing();
		}
	}

	@Override
	public void updateWaves(Bullet bullet) {
		// TODO WSMovement
		if (robot.getOthers() <= 1) {
			// If the enemyWaves collection is empty, we must have missed the
			// detection of this wave somehow.
			if (!enemyWaves.isEmpty()) {
				Point2D.Double hitBulletLocation = new Point2D.Double(bullet.getX(), bullet.getY());
				EnemyWave hitWave = null;

				// look through the EnemyWaves, and find the one that could've
				// hit
				// us.
				for (int x = 0; x < enemyWaves.size(); x++) {
					EnemyWave ew = enemyWaves.get(x);

					if (Math.abs(ew.distanceTraveled - myLocation.distance(ew.fireLocation)) < 50
							&& Math.abs(RoboUtils.bulletVelocity(bullet.getPower()) - ew.bulletVelocity) < 0.001) {
						hitWave = ew;
						break;
					}
				}
				if (null != hitWave) {
					logHit(hitWave, hitBulletLocation);
					// We can remove this wave now
					enemyWaves.remove(enemyWaves.lastIndexOf(hitWave));
				}
			}
		}
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
						* (1 + Math.abs(Math.cos(RoboUtils.absoluteBearing(point, info.myLocation)
								- RoboUtils.absoluteBearing(point, enemy.position))))
						/ point.distanceSq(enemy.position);
			}
		}

		return eval;
	}

	private void doSurfing() {
		EnemyWave surfWave = getClosestSurfableWave();

		if (null == surfWave) {
			return;
		}

		double dangerLeft = checkDanger(surfWave, -1);
		double dangerRight = checkDanger(surfWave, 1);

		double goAngle = RoboUtils.absoluteBearing(surfWave.fireLocation, myLocation);
		if (dangerLeft < dangerRight) {
			goAngle = wallSmoothing(myLocation, goAngle - LESS_THAN_HALF_PI, -1);
		} else {
			goAngle = wallSmoothing(myLocation, goAngle + LESS_THAN_HALF_PI, 1);
		}

		setBackAsFront(robot, goAngle);
	}

	private void updateWaves() {
		for (int x = 0; x < enemyWaves.size(); x++) {
			EnemyWave enemyWave = enemyWaves.get(x);
			enemyWave.distanceTraveled = (robot.getTime() - enemyWave.fireTime) * enemyWave.bulletVelocity;
			if (enemyWave.distanceTraveled > myLocation.distance(enemyWave.fireLocation) + 50) {
				enemyWaves.remove(x);
				x--;
			}
		}
	}

	private EnemyWave getClosestSurfableWave() {
		double closestDistance = 50000; // Very large distance
		EnemyWave surfWave = null;

		for (int x = 0; x < enemyWaves.size(); x++) {
			EnemyWave wave = enemyWaves.get(x);
			double distance = myLocation.distance(wave.fireLocation) - wave.distanceTraveled;

			if (distance > wave.bulletVelocity && distance < closestDistance) {
				surfWave = wave;
				closestDistance = distance;
			}
		}

		return surfWave;
	}

	private double checkDanger(EnemyWave wave, int direction) {
		int index = getFactorIndex(wave, predictPosition(wave, direction));
		return surfStats[index];
	}

	private static void setBackAsFront(AdvancedRobot robot, double goAngle) {
		double angle = Utils.normalRelativeAngle(goAngle - robot.getHeadingRadians());
		if (Math.abs(angle) > (Math.PI / 2)) {
			if (angle < 0) {
				robot.setTurnRightRadians(Math.PI + angle);
			} else {
				robot.setTurnLeftRadians(Math.PI - angle);
			}
			robot.setBack(100);
		} else {
			if (angle < 0) {
				robot.setTurnLeftRadians(-1 * angle);
			} else {
				robot.setTurnRightRadians(angle);
			}
			robot.setAhead(100);
		}
	}

	private Point2D.Double predictPosition(EnemyWave wave, int direction) {
		Point2D.Double predictedPosition = (Point2D.Double) myLocation.clone();
		double predictedVelocity = robot.getVelocity();
		double predictedHeading = robot.getHeadingRadians();
		double maxTurning, moveAngle, moveDir;

		int counter = 0; // number of ticks in the future
		boolean intercepted = false;

		while (!intercepted && counter < 500) {
			moveAngle = wallSmoothing(predictedPosition,
					RoboUtils.absoluteBearing(wave.fireLocation, predictedPosition) + (direction * (Math.PI / 2)),
					direction) - predictedHeading;
			moveDir = 1;

			if (Math.cos(moveAngle) < 0) {
				moveAngle += Math.PI;
				moveDir = -1;
			}

			moveAngle = Utils.normalRelativeAngle(moveAngle);

			// maxTurning is built in like this, you can't turn more than this
			// in one tick
			maxTurning = Math.PI / 720d * (40d - 3d * Math.abs(predictedVelocity));
			predictedHeading = Utils
					.normalRelativeAngle(predictedHeading + RoboUtils.limit(-maxTurning, moveAngle, maxTurning));
			// If predictedVelocity and moveDir have
			// different signs you want to break down
			// otherwise you want to accelerate (look at the factor "2")
			predictedVelocity += (predictedVelocity * moveDir < 0 ? 2 * moveDir : moveDir);
			predictedVelocity = RoboUtils.limit(-8, predictedVelocity, 8);

			// calculate the new predicted position
			predictedPosition = RoboUtils.project(predictedPosition, predictedHeading, predictedVelocity);

			counter++;

			if (predictedPosition.distance(wave.fireLocation) < wave.distanceTraveled + (counter * wave.bulletVelocity)
					+ wave.bulletVelocity) {
				intercepted = true;
			}
		}

		return predictedPosition;
	}

	private double wallSmoothing(Point2D.Double botLocation, double angle, int direction) {
		while (!fieldRectangle.contains(RoboUtils.project(botLocation, angle, WALL_STICK))) {
			angle += direction * 0.05;
		}
		return angle;
	}

	// Given the EnemyWave that the bullet was on, and the point where we
	// were hit, calculate the index into our stat array for that factor.
	private static int getFactorIndex(EnemyWave wave, Point2D.Double targetLocation) {
		double offsetAngle = (RoboUtils.absoluteBearing(wave.fireLocation, targetLocation) - wave.directAngle);
		double factor = Utils.normalRelativeAngle(offsetAngle) / RoboUtils.maxEscapeAngle(wave.bulletVelocity)
				* wave.direction;

		return (int) RoboUtils.limit(0, (factor * ((BINS - 1) / 2)) + ((BINS - 1) / 2), BINS - 1);
	}

	private void logHit(EnemyWave wave, Point2D.Double targetLocation) {
		int index = getFactorIndex(wave, targetLocation);

		for (int x = 0; x < BINS; x++) {
			// for the spot bin that we were hit on, add 1;
			// for the bins next to it, add 1/2;
			// for the next one, add 1/5, and so on...
			surfStats[x] += 1.0 / (Math.pow(index - x, 2) + 1);
		}
	}

}
