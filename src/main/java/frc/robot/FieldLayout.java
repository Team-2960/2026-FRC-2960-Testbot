package frc.robot;

import static edu.wpi.first.units.Units.Degrees;
import static edu.wpi.first.units.Units.Inches;
import static edu.wpi.first.units.Units.Meters;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.units.measure.Angle;
import edu.wpi.first.units.measure.Distance;
import edu.wpi.first.wpilibj.DriverStation;

public class FieldLayout {
    // Overall Field Dimensions
    public static final Distance fieldSizeX = Inches.of(651.22);
    public static final Distance fieldSizeY = Inches.of(317.69);

    public static final Distance fieldCenterX = fieldSizeX.div(2);
    public static final Distance fieldCenterY = fieldSizeY.div(2);

    public static final Angle blueForwardAngle = Degrees.of(0);
    public static final Angle redForwardAngle = Degrees.of(180);

    // Start Line Dimensions
    public static final Distance startLineToWallOffset = Inches.of(156.61);
    public static final Distance startLineWidth = Inches.of(2);

    // Hub Dimensions
    public static final Distance hubXToCenterOffset = Inches.of(143.5);
    public static final Distance hubBaseWidth = Inches.of(47);
    public static final Distance hubFunnelDiam = Inches.of(41.5);

    public static final Distance blueHubCenterX = fieldCenterY.minus(hubXToCenterOffset);
    public static final Distance redHubCenterX = fieldCenterX.plus(hubXToCenterOffset);

    public static final Distance blueHubLeftY = fieldCenterY.plus(hubBaseWidth.div(2));
    public static final Distance blueHubRightY = fieldCenterY.minus(hubBaseWidth.div(2));

    public static final Distance redHubLeftY = blueHubRightY;
    public static final Distance redHubRightY = blueHubLeftY;

    public static final Distance blueHubFront = startLineToWallOffset.plus(startLineWidth);
    public static final Distance redHubFront = fieldSizeX.minus(blueHubFront);

    public static final Translation2d blueHubCenter = new Translation2d(blueHubCenterX, fieldCenterY);
    public static final Translation2d redHubCenter = new Translation2d(redHubCenterX, fieldCenterX);

    public static final Translation2d blueHubCenterFront = new Translation2d(blueHubFront, fieldCenterY);
    public static final Translation2d redHubCenterFront = new Translation2d(blueHubFront, fieldCenterY);

    /**
     * Checks if the current alliance is red.
     * 
     * @return True if the current alliance is Red. False if alliance is Blue or the
     *         alliance is not set
     */
    public static boolean isRedAlliance() {
        var alliance = DriverStation.getAlliance();
        return alliance.isPresent() && alliance.get() == DriverStation.Alliance.Red;
    }

    /**
     * Gets the field orientation that is considered forward for the current
     * alliance.
     * 
     * @return forward angle for the current alliance. Defaults to blue if alliance
     *         is not set.
     */
    public static Angle getForwardAngle() {
        return isRedAlliance() ? redForwardAngle : blueForwardAngle;
    }

    /**
     * Gets the center of the hub for the current alliance
     * 
     * @return center of the hub for the current alliance. Defaults to blue if
     *         alliance is not set.
     */
    public static Translation2d getHubCenter() {
        return isRedAlliance() ? redHubCenter : blueHubCenter;
    }

    /**
     * Gets the front center of the hub for the current alliance
     * 
     * @return front center of the hub for the current alliance. Defaults to blue if
     *         alliance is not set.
     */
    public static Translation2d getHubCenterFront() {
        return isRedAlliance() ? redHubCenterFront : blueHubCenterFront;
    }

    /**
     * Calculates the distance to the current alliance hub from a given position
     * @param position  position to check to distance of
     * @return  distance to the current alliance hub 
     */
    public static Distance getHubDist(Translation2d position) {
        return Meters.of(position.getDistance(FieldLayout.getHubCenter()));
    }
}
