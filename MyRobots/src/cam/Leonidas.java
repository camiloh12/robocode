package cam;

import java.awt.Color;
import java.awt.geom.Point2D;
import java.awt.geom.Rectangle2D;
import java.util.ArrayList;
import java.util.Enumeration;
import java.util.Hashtable;
import java.util.List;

import robocode.AdvancedRobot;
import robocode.Bullet;
import robocode.BulletHitBulletEvent;
import robocode.Condition;
import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.RobotDeathEvent;
import robocode.ScannedRobotEvent;
import robocode.WinEvent;
import robocode.util.Utils;

/**
 * King of Sparta
 * 
 * This robot is able to switch between melee and 1v1 strategies based on the
 * number of opponents on the field.
 * 
 * The 1v1 robot is based on BasicGFSurfer by Voidious, PEZ, and Bayen.
 * 
 * It is a combination of the movement from BasicSurfer and the gun from
 * GFTargetingBot. See: http://robowiki.net?BasicSurfer
 * http://robowiki.net?GFTargetingBot
 * 
 * The melee version is based on HawkOnFire by Rozu. See:
 * http://robowiki.net?HawkOnFire
 * 
 * @author camilo
 *
 */
public class Leonidas extends AdvancedRobot {

	private Movement movement;
	private Gun gun;

	// This info is only used during melee
	public BattleInfo info;

	public Leonidas() {
		info = new BattleInfo();
	}

	public void run() {
		setColors(Color.BLACK, Color.BLACK, Color.BLACK, Color.RED, Color.RED);

		setAdjustGunForRobotTurn(true);
		setAdjustRadarForGunTurn(true);

		setTurnRadarRightRadians(Double.POSITIVE_INFINITY);

		info.nextDestination = info.lastPosition = info.myLocation = new Point2D.Double(
				getX(), getY());
		info.target = new Enemy();
		
		System.out.println("Spartans, prepare for glory!");

		while (true) {
			// If there is more than 1 opponent use the
			// Head-On Targeting gun and Minimum Risk Movement
			if (getOthers() > 1) {
				gun = HOTGun.getGun(this);
				((HOTGun) gun).info = info;
				movement = MRMovement.getMovement(this);
				((MRMovement) movement).info = info;

				info.myLocation = new Point2D.Double(getX(), getY());
				info.myEnergy = getEnergy();

				// wait until you have scanned all other bots.
				// this should take around 7 to 9 ticks.
				if (info.target.live && getTime() > 9) {
					movement.update();
					gun.update();
				}
			}
			// Else use the Guess Factor Targeting gun
			// and Wave Surfing movement
			else {
				gun = GFTGun.getGun(this);
				movement = WSMovement.getMovement(this);
				gun.update();

				if (gun.getLastScanTime() + 1 < getTime()) {
					// No scans, forcing surfing
					movement.update();
				}
			}
			execute();
		}
	}

	public void onScannedRobot(ScannedRobotEvent event) {
		movement.onScannedRobot(event);
		gun.onScannedRobot(event);
	}

	public void onHitRobot(HitRobotEvent event) {
		// If he's in front of us, back up
		if (event.getBearing() > -90 && event.getBearing() < 90) {
			setBack(100);
		} // else he's in back of us, set ahead
		else {
			setAhead(100);
		}
	}

	public void onHitByBullet(HitByBulletEvent event) {
		movement.updateWaves(event.getBullet());
	}

	public void onBulletHitByBullet(BulletHitBulletEvent event) {
		movement.updateWaves(event.getBullet());
	}

	public void onRobotDeath(RobotDeathEvent event) {
		Enemy enemy = info.enemies.get(event.getName());
		if (null != enemy) {
			enemy.live = false;
		}
	}

	public void onWin(WinEvent event) {
		setTurnRadarRightRadians(Double.POSITIVE_INFINITY);
		setTurnGunLeftRadians(Double.POSITIVE_INFINITY);
		for (int i = 0; i < 50; i++) {
			turnRight(35);
			turnLeft(35);
		}
	}
}

interface Movement {

	public void onScannedRobot(ScannedRobotEvent event);

	public void update();

	public void updateWaves(Bullet bullet);
}

interface Gun {

	public void onScannedRobot(ScannedRobotEvent event);

	public void update();

	public long getLastScanTime();
}

/**
 * Guess Factor Targeting Gun
 * 
 * This gun is best suited for 1v1
 *
 */
class GFTGun implements Gun {

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

/**
 * Head-On Targeting Gun
 * 
 * This gun is best suited for melee
 *
 */
class HOTGun implements Gun {

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
		enemy.position = RoboUtils.project(info.myLocation,
				robot.getHeadingRadians() + event.getBearingRadians(),
				event.getDistance());

		// normal target selection: the one closer to you is the most dangerous
		// so attack him
		if (!info.target.live
				|| event.getDistance() < info.myLocation
						.distance(info.target.position)) {
			info.target = enemy;
		}

		// locks the radar if there is only one opponent left
		if (robot.getOthers() == 1) {
			robot.setTurnRadarLeftRadians(robot.getRadarTurnRemainingRadians());
		} else {
			robot.setTurnGunRightRadians(Utils.normalRelativeAngle(RoboUtils
					.absoluteBearing(info.myLocation, info.target.position)
					- robot.getGunHeadingRadians()));
		}
	}

	@Override
	public void update() {
		// HeadOnTargeting
		double distanceToTarget = info.myLocation
				.distance(info.target.position);
		if (robot.getGunTurnRemainingRadians() == 0 && info.myEnergy > 1) {
			robot.setFire(Math.min(
					Math.min(info.myEnergy / 6d, 1300d / distanceToTarget),
					info.target.energy / 3d));
		}
	}

	@Override
	public long getLastScanTime() {
		return info.lastScanTime;
	}

}

/**
 * Wave Surfing Movement class
 * 
 * This movement is best suited for 1v1
 *
 */
class WSMovement implements Movement {

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
	private static WSMovement movement;

	private AdvancedRobot robot;
	// Wall Smoothing
	private Rectangle2D.Double fieldRectangle;
	private Point2D.Double myLocation;
	private Point2D.Double enemyLocation;

	private List<EnemyWave> enemyWaves = new ArrayList<>();
	private List<Integer> surfDirections = new ArrayList<>();
	private List<Double> surfAbsBearings = new ArrayList<>();

	private WSMovement(AdvancedRobot robot) {
		this.robot = robot;
		fieldRectangle = new Rectangle2D.Double(WALL_MARGIN, WALL_MARGIN,
				robot.getBattleFieldWidth() - WALL_MARGIN * 2,
				robot.getBattleFieldHeight() - WALL_MARGIN * 2);
	}

	public static WSMovement getMovement(AdvancedRobot robot) {
		if (null == movement || robot.getTime() == 0) {
			movement = new WSMovement(robot);
		}
		return movement;
	}

	@Override
	public void onScannedRobot(ScannedRobotEvent event) {
		myLocation = new Point2D.Double(robot.getX(), robot.getY());

		double lateralVelocity = robot.getVelocity()
				* Math.sin(event.getBearingRadians());
		double absBearing = event.getBearingRadians()
				+ robot.getHeadingRadians();

		robot.setTurnRadarRightRadians(Utils.normalRelativeAngle(absBearing
				- robot.getRadarHeadingRadians() * 2));

		surfDirections.add(0, new Integer((lateralVelocity >= 0) ? 1 : -1));
		surfAbsBearings.add(0, new Double(absBearing + Math.PI));

		double bulletPower = oppEnergy - event.getEnergy();
		if (bulletPower < 3.01 && bulletPower > 0.09
				&& surfDirections.size() > 2) {
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
		enemyLocation = RoboUtils.project(myLocation, absBearing,
				event.getDistance());

		updateWaves();
		doSurfing();
	}

	private void doSurfing() {
		EnemyWave surfWave = getClosestSurfableWave();

		if (null == surfWave) {
			return;
		}

		double dangerLeft = checkDanger(surfWave, -1);
		double dangerRight = checkDanger(surfWave, 1);

		double goAngle = RoboUtils.absoluteBearing(surfWave.fireLocation,
				myLocation);
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
			enemyWave.distanceTraveled = (robot.getTime() - enemyWave.fireTime)
					* enemyWave.bulletVelocity;
			if (enemyWave.distanceTraveled > myLocation
					.distance(enemyWave.fireLocation) + 50) {
				enemyWaves.remove(x);
				x--;
			}
		}
	}

	@Override
	public void update() {
		myLocation = new Point2D.Double(robot.getX(), robot.getY());
		updateWaves();
		doSurfing();
	}

	@Override
	public void updateWaves(Bullet bullet) {
		// If the enemyWaves collection is empty, we must have missed the
		// detection of this wave somehow.
		if (!enemyWaves.isEmpty()) {
			Point2D.Double hitBulletLocation = new Point2D.Double(
					bullet.getX(), bullet.getY());
			EnemyWave hitWave = null;

			// look through the EnemyWaves, and find the one that could've hit
			// us.
			for (int x = 0; x < enemyWaves.size(); x++) {
				EnemyWave ew = enemyWaves.get(x);

				if (Math.abs(ew.distanceTraveled
						- myLocation.distance(ew.fireLocation)) < 50
						&& Math.abs(RoboUtils.bulletVelocity(bullet.getPower())
								- ew.bulletVelocity) < 0.001) {
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

	private EnemyWave getClosestSurfableWave() {
		double closestDistance = 50000; // Very large distance
		EnemyWave surfWave = null;

		for (int x = 0; x < enemyWaves.size(); x++) {
			EnemyWave wave = enemyWaves.get(x);
			double distance = myLocation.distance(wave.fireLocation)
					- wave.distanceTraveled;

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
		double angle = Utils.normalRelativeAngle(goAngle
				- robot.getHeadingRadians());
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
			moveAngle = wallSmoothing(
					predictedPosition,
					RoboUtils.absoluteBearing(wave.fireLocation,
							predictedPosition) + (direction * (Math.PI / 2)),
					direction)
					- predictedHeading;
			moveDir = 1;

			if (Math.cos(moveAngle) < 0) {
				moveAngle += Math.PI;
				moveDir = -1;
			}

			moveAngle = Utils.normalRelativeAngle(moveAngle);

			// maxTurning is built in like this, you can't turn more than this
			// in one tick
			maxTurning = Math.PI / 720d
					* (40d - 3d * Math.abs(predictedVelocity));
			predictedHeading = Utils.normalRelativeAngle(predictedHeading
					+ RoboUtils.limit(-maxTurning, moveAngle, maxTurning));
			// If predictedVelocity and moveDir have
			// different signs you want to break down
			// otherwise you want to accelerate (look at the factor "2")
			predictedVelocity += (predictedVelocity * moveDir < 0 ? 2 * moveDir
					: moveDir);
			predictedVelocity = RoboUtils.limit(-8, predictedVelocity, 8);

			// calculate the new predicted position
			predictedPosition = RoboUtils.project(predictedPosition,
					predictedHeading, predictedVelocity);

			counter++;

			if (predictedPosition.distance(wave.fireLocation) < wave.distanceTraveled
					+ (counter * wave.bulletVelocity) + wave.bulletVelocity) {
				intercepted = true;
			}
		}

		return predictedPosition;
	}

	private double wallSmoothing(Point2D.Double botLocation, double angle,
			int direction) {
		while (!fieldRectangle.contains(RoboUtils.project(botLocation, angle,
				WALL_STICK))) {
			angle += direction * 0.05;
		}
		return angle;
	}

	// Given the EnemyWave that the bullet was on, and the point where we
	// were hit, calculate the index into our stat array for that factor.
	private static int getFactorIndex(EnemyWave wave,
			Point2D.Double targetLocation) {
		double offsetAngle = (RoboUtils.absoluteBearing(wave.fireLocation,
				targetLocation) - wave.directAngle);
		double factor = Utils.normalRelativeAngle(offsetAngle)
				/ RoboUtils.maxEscapeAngle(wave.bulletVelocity)
				* wave.direction;

		return (int) RoboUtils.limit(0, (factor * ((BINS - 1) / 2))
				+ ((BINS - 1) / 2), BINS - 1);
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

/**
 * Minimum Risk Movement class
 * 
 * This movement is best suited for melee
 *
 */
class MRMovement implements Movement {

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

/**
 * Set of utility methods used for various calculations
 */
class RoboUtils {

	public static double bulletVelocity(double power) {
		return 20 - 3 * power;
	}

	public static Point2D.Double project(Point2D.Double sourceLocation,
			double angle, double length) {
		return new Point2D.Double(sourceLocation.x + Math.sin(angle) * length,
				sourceLocation.y + Math.cos(angle) * length);
	}

	public static double absoluteBearing(Point2D.Double source,
			Point2D.Double target) {
		return Math.atan2(target.x - source.x, target.y - source.y);
	}

	public static int sign(double v) {
		return v < 0 ? -1 : 1;
	}

	public static int minMax(int v, int min, int max) {
		return Math.max(min, Math.min(max, v));
	}

	public static double limit(double min, double value, double max) {
		return Math.max(min, Math.min(value, max));
	}

	public static double maxEscapeAngle(double velocity) {
		return Math.asin(8.0 / velocity);
	}
}

/**
 * Guess Factor targeting wave
 */
class GFTWave extends Condition {

	private static final double MAX_DISTANCE = 1000;
	private static final int DISTANCE_INDEXES = 5;
	private static final int VELOCITY_INDEXES = 5;
	private static final int BINS = 25;
	private static final int MIDDLE_BIN = (BINS - 1) / 2;
	private static final double MAX_ESCAPE_ANGLE = 0.7;
	private static final double BIN_WIDTH = MAX_ESCAPE_ANGLE
			/ (double) MIDDLE_BIN;

	private static int[][][][] statBuffers = new int[DISTANCE_INDEXES][VELOCITY_INDEXES][VELOCITY_INDEXES][BINS];

	private AdvancedRobot robot;
	private int[] buffer;
	private double distanceTraveled;

	public static Point2D.Double targetLocation;

	public Point2D.Double gunLocation;
	public double bulletPower;
	public double bearing;
	public double lateralDirection;

	public GFTWave(AdvancedRobot robot) {
		this.robot = robot;
	}

	@Override
	public boolean test() {
		advance();
		if (hasArrived()) {
			buffer[currentBin()]++;
			robot.removeCustomEvent(this);
		}
		return false;
	}

	public double mostVisitedBearingOffset() {
		return (lateralDirection * BIN_WIDTH) * (mostVisitedBin() - MIDDLE_BIN);
	}

	public void setSegmentations(double distance, double velocity,
			double lastVelocity) {
		int distanceIndex = Math.min(DISTANCE_INDEXES - 1,
				(int) (distance / (MAX_DISTANCE / DISTANCE_INDEXES)));
		int velocityIndex = (int) Math.abs(velocity / 2);
		int lastVelocityIndex = (int) Math.abs(lastVelocity / 2);
		buffer = statBuffers[distanceIndex][velocityIndex][lastVelocityIndex];
	}

	private void advance() {
		distanceTraveled += RoboUtils.bulletVelocity(bulletPower);
	}

	private boolean hasArrived() {
		return distanceTraveled > gunLocation.distance(targetLocation) - 18;

	}

	private int currentBin() {
		int bin = (int) Math
				.round(((Utils.normalRelativeAngle(RoboUtils.absoluteBearing(
						gunLocation, targetLocation) - bearing)) / (lateralDirection * BIN_WIDTH))
						+ MIDDLE_BIN);
		return RoboUtils.minMax(bin, 0, BINS - 1);
	}

	private int mostVisitedBin() {
		int mostVisited = MIDDLE_BIN;
		for (int i = 0; i < BINS; i++) {
			if (buffer[i] > buffer[mostVisited]) {
				mostVisited = i;
			}
		}
		return mostVisited;
	}

}

class BattleInfo {
	public Hashtable<String, Enemy> enemies = new Hashtable<>();
	public Enemy target;
	public Point2D.Double myLocation;
	public Point2D.Double nextDestination;
	public Point2D.Double lastPosition;
	public long lastScanTime;
	public double myEnergy;
}

class EnemyWave {
	public Point2D.Double fireLocation;
	public long fireTime;
	public double bulletVelocity;
	public double directAngle;
	public double distanceTraveled;
	public int direction;
}

class Enemy {
	public Point2D.Double position;
	public double energy;
	public boolean live;
}