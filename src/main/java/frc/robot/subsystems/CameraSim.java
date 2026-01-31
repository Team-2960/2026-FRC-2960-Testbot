package frc.robot.subsystems;

import org.photonvision.simulation.VisionSystemSim;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInLayouts;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class CameraSim extends SubsystemBase{

    private Field2d cameraField;

    private CommandSwerveDrivetrain drivetrain;

    //Simulation
    private VisionSystemSim visionSim = new VisionSystemSim("main");

    /**
     * Constructor
     * @param drivetrain    drivetrain object reference
     * @param cameras       list of cameras
     * @param rightCamera
     */
    public CameraSim(CommandSwerveDrivetrain drivetrain, AprilTagPipeline... cameras){
        
        this.drivetrain = drivetrain;

        // Layout
        var layout = Shuffleboard.getTab("AprilTags")
                .getLayout("Camera Update", BuiltInLayouts.kList)
                .withSize(1, 4);

        cameraField = new Field2d();
        layout.add(cameraField).withWidget("Field");

        // Initialize simulation
        visionSim.addAprilTags(AprilTagFieldLayout.loadField(AprilTagFields.kDefaultField));

        for(var camera : cameras) visionSim.addCamera(camera.getCameraSim(), camera.getOffset());
        
    }

    /**
     * Update the simulation
     */
    @Override
    public void simulationPeriodic(){
        visionSim.update(drivetrain.getPose2d());
    }
}
