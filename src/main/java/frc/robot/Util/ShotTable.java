package frc.robot.Util;

import edu.wpi.first.math.interpolation.InterpolatingTreeMap;
import edu.wpi.first.units.DistanceUnit;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;
import edu.wpi.first.units.measure.Distance;

public class ShotTable<U extends Unit> {
    private final InterpolatingTreeMap<Measure<DistanceUnit>, Measure<U>> shotTable = new InterpolatingTreeMap<Measure<DistanceUnit>, Measure<U>>(
            new UnitInverseInterpolator<>(),
            new UnitInterpolator<>());

    /**
     * Adds an entry to the shot table
     * 
     * @param dist  distance for the entry
     * @param value value for the entry
     * @return Reference to the current object for method chaining
     */
    public ShotTable<U> addEntry(Distance dist, Measure<U> value) {
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
    public Measure<U> get(Distance dist) {
        return shotTable.get(dist);
    }
}
