package cam.model;

import java.awt.geom.Point2D;
import java.util.Hashtable;

public class BattleInfo {
	public Hashtable<String, Enemy> enemies = new Hashtable<>();
	public Enemy target;
	public Point2D.Double myLocation;
	public Point2D.Double nextDestination;
	public Point2D.Double lastPosition;
	public long lastScanTime;
	public double myEnergy;
}
