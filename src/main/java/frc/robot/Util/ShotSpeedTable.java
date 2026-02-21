package frc.robot.Util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.Distance;

public class ShotSpeedTable {
    private final InterpolatingTreeMap<Distance, AngularVelocity> shotTable = new InterpolatingTreeMap<>(
            new DistInverseInterpolator(),
            new AngVelInterpolator());

    /**
     * Adds an entry to the shot table
     * 
     * @param dist  distance for the entry
     * @param value value for the entry
     * @return Reference to the current object for method chaining
     */
    public ShotSpeedTable addEntry(Distance dist, AngularVelocity value) {
        shotTable.put(dist, value);
        return this;
    }

    /**
     * Returns the value associated with a given distance.
     * 
     * If there's no matching distance, the value returned will be an interpolation
     * between the distances before and after the provided one.
     * 
     * @param dist distance to check
     * @return value associated with the distance
     */
    public AngularVelocity get(Distance dist) {
        return shotTable.get(dist);
    }
}
