package frc.robot.subsystems;

import au.grapplerobotics.LaserCan;
import static edu.wpi.first.units.Units.Meters;
import edu.wpi.first.units.measure.Distance;

public class LaserCAN {
    private LaserCan laserCan;
    public Distance distance;

    public LaserCAN(int CANID) {
        laserCan = new LaserCan(CANID);
    }

    public Distance getDistance() {
        return Meters.of(laserCan.getMeasurement().distance_mm / 1000.0);
    }
}
