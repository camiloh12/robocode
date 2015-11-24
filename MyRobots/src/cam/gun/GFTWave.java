package cam.gun;

import java.awt.geom.Point2D;

import cam.utils.RoboUtils;
import robocode.AdvancedRobot;
import robocode.Condition;
import robocode.util.Utils;

/**
 * Guess Factor targeting wave
 */
public class GFTWave extends Condition {
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
