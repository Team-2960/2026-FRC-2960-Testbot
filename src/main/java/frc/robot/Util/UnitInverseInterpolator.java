package frc.robot.Util;

import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.Measure;
import edu.wpi.first.units.Unit;

/**
 * Inverse interpolator for unit measures
 */
public class UnitInverseInterpolator <U extends Unit> implements InverseInterpolator<Measure<U>> {

    @Override
    public double inverseInterpolate(Measure<U> startValue, Measure<U> endValue, Measure<U> q) {
           return endValue.mutableCopy().minus(startValue).div(q).magnitude();
    }
}
