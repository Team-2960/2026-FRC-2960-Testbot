package frc.robot.Util;

import edu.wpi.first.math.interpolation.Interpolator;
import edu.wpi.first.units.measure.AngularVelocity;


/**
 * Interpolator for unit measures
 */
public class AngVelInterpolator implements Interpolator<AngularVelocity>{
    @Override
    public AngularVelocity interpolate(AngularVelocity startValue, AngularVelocity endValue, double t) {
        return endValue.mutableCopy().minus(startValue).times(t).plus(startValue);
    }

}
