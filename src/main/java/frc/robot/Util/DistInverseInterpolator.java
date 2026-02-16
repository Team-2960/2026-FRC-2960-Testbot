package frc.robot.Util;

import edu.wpi.first.math.interpolation.InverseInterpolator;
import edu.wpi.first.units.measure.Distance;

/**
 * Inverse interpolator for unit measures
 */
public class DistInverseInterpolator implements InverseInterpolator<Distance> {

    @Override
    public double inverseInterpolate(Distance startValue, Distance endValue, Distance q) {
           return endValue.mutableCopy().minus(startValue).div(q).magnitude();
    }
}
