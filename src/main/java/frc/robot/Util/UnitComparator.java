package frc.robot.Util;

import java.util.Comparator;

import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Comparator for unit measures
 */
public class UnitComparator <U extends Unit> implements Comparator<Measure<U>> {

    @Override
    public int compare(Measure<U> o1, Measure<U> o2) {
        return o1.compareTo(o2);
    }
    
}
