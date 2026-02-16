package frc.robot.Util;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;


/**
 * Interpolator for unit measures
 */
public class UnitInterpolator<U extends Unit> implements Interpolator<Measure<U>>{
    @Override
    public Measure<U> interpolate(Measure<U> startValue, Measure<U> endValue, double t) {
        return endValue.mutableCopy().minus(startValue).times(t).plus(startValue);
    }

}
