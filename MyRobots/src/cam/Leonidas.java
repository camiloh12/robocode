package cam;

import java.awt.Color;
import java.awt.geom.Point2D;

import cam.gun.GFTGun;
import cam.gun.Gun;
import cam.gun.HOTGun;
import cam.model.BattleInfo;
import cam.model.Enemy;
import cam.movement.CombinedMovement;
import robocode.AdvancedRobot;
import robocode.BulletHitBulletEvent;
import robocode.HitByBulletEvent;
import robocode.HitRobotEvent;
import robocode.RobotDeathEvent;
import robocode.ScannedRobotEvent;
import robocode.WinEvent;

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

	private CombinedMovement movement;
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

		info.nextDestination = info.lastPosition = info.myLocation = new Point2D.Double(getX(), getY());
		info.target = new Enemy();

		System.out.println("Spartans, prepare for glory!");
		movement = CombinedMovement.getMovement(this);
		movement.info = info;

		while (true) {
			// If there is more than 1 opponent use the
			// Head-On Targeting gun and Minimum Risk Movement
			if (getOthers() > 1) {
				gun = HOTGun.getGun(this);
				((HOTGun) gun).info = info;

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
