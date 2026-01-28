package frc.robot.Util.CustomSwerveRequests;

import java.util.function.Supplier;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveControlParameters;
import com.ctre.phoenix6.swerve.SwerveRequest.FieldCentricFacingAngle;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.RobotContainer;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveRequest;

public class FieldCenterLookAtPoint extends FieldCentricFacingAngle{

    public FieldCentricFacingAngle withTargetPoint(Translation2d targetPoint, Supplier<Pose2d> currentPose){
        
        Translation2d calcPoint = targetPoint.minus(currentPose.get().getTranslation());
        Rotation2d calcRot = calcPoint.getAngle();

        return this.withTargetDirection(calcRot);
    }
}

