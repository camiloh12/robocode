package cam.model;

import java.awt.geom.Point2D;

public class Enemy {
	public Point2D.Double position;
	public double energy;
	public boolean live;
	public double velocity;
	public double heading;
	public double lateralVelocity;
	public double oldHeading;
	public double headingChange;
	public double bearing;
}
