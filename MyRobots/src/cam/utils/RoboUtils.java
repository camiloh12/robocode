package cam.utils;

import java.awt.geom.Point2D;

/**
 * Set of utility methods used for various calculations
 */
public class RoboUtils {

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
