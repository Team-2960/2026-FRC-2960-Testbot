package frc.robot.subsystems;

import org.photonvision.PhotonPoseEstimator.PoseStrategy;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Util.AprilTagPipelineSettings;

public class Cameras extends SubsystemBase{
    private AprilTagPipelineSettings leftPipeline;
    private AprilTagPipelineSettings rightPipeline;
    private edu.wpi.first.math.Vector<N3> singleStds;
    private edu.wpi.first.math.Vector<N3> multiStds;

    public AprilTagPipeline frontCamera;
    public AprilTagPipeline rightCamera;
    public AprilTagPipeline leftCamera;
    private Field2d cameraField;

    //Simulation
    // private VisionSystemSim visionSim;

    public Cameras(CommandSwerveDrivetrain drive){

        singleStds = VecBuilder.fill(4, 4, 16);
        multiStds = VecBuilder.fill(0.5, 0.5, 1);

        leftPipeline = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
            Constants.leftCameraOffsets,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            3, 
            singleStds , 
            multiStds,
            .2
        );

        rightPipeline = new AprilTagPipelineSettings(AprilTagFields.k2026RebuiltWelded,
            Constants.rightCameraOffsets,
            PoseStrategy.MULTI_TAG_PNP_ON_COPROCESSOR,
            3, 
            singleStds , 
            multiStds,
            .2
        );
        
        leftCamera = new AprilTagPipeline(drive, leftPipeline, "LeftCamera", "LeftCamera");
        rightCamera = new AprilTagPipeline(drive, rightPipeline, "RightCamera", "RightCamera");

        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout("Camera Update", BuiltInLayouts.kList)
                .withSize(1, 4);

        cameraField = new Field2d();
        layout.add(cameraField).withWidget("Field");

        //Simulation
        // visionSim = new VisionSystemSim("main");
        // visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.k2025ReefscapeWelded));

        // visionSim.addCamera(frontCamera.cameraSim, frontPipeline.robot_to_camera);
        // visionSim.addCamera(leftCamera.cameraSim, leftPipeline.robot_to_camera);
        // visionSim.addCamera(rightCamera.cameraSim, rightPipeline.robot_to_camera);
        
    }

    public void updateUI(){
    }

    @Override
    public void periodic(){
        // visionSim.update(Drive.getInstance().getEstimatedPos());
    }
}
