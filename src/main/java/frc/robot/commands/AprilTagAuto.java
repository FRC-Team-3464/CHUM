package frc.robot.commands;


    import org.photonvision.PhotonCamera;
    import org.photonvision.targeting.PhotonPipelineResult;
    import org.photonvision.targeting.PhotonTrackedTarget;

    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
    import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
    import edu.wpi.first.wpilibj2.command.WaitCommand;
    import frc.robot.commands.Armevator.ArmevatorToPosition;
    import frc.robot.commands.Armevator.RunElevator;
    import frc.robot.commands.Coral.ReverseCoralIntake;
    import frc.robot.subsystems.ArmSubsystem;
    import frc.robot.subsystems.CoralSubsystem;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
    import frc.robot.subsystems.VisionSubsystem;

    
    public class AprilTagAuto extends Command{
  
        // !! this value needs to be checked and updated
        private static final double HALF_BRANCH_DIFF = 0.1;     
         
         // camera name
        private static final int FRONT_CAMERA = 1; 
        private static final int BACK_CAMERA = 2; 

        private PhotonCamera camera;
        private SwerveSubsystem swerveSub;
        private VisionSubsystem visionSub;
    
        private final DriveForwardAuto driveSub = new DriveForwardAuto();
        private final CoralSubsystem coralSub = new CoralSubsystem();
        
        
        public  AprilTagAuto (int iAutoStage) {
            this.swerveSub = SwerveSubsystem.getInstance();
            this.visionSub = VisionSubsystem.getInstance();

            if (iAutoStage == FRONT_CAMERA) {
                this.camera = visionSub.frontAprilCamera;
                System.out.println("1-link to front camera");
            } else if (iAutoStage == BACK_CAMERA) {
                this.camera = visionSub.backAprilCamera;
                System.out.println("2-link to front camera");
            } else {
                this.camera = visionSub.frontAprilCamera;
                System.out.println("3-link to front camera");
            }

        }
    
        // iAutoStage:1/2-to branch/coral pickup; iBranch: 1/-1-left or right for coral drop
        public void processAutoAccess(int iAutoStage, int iAdjust) {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {
                PhotonTrackedTarget target = result.getBestTarget();
                double targetX = target.getBestCameraToTarget().getX();
                double targetY = target.getBestCameraToTarget().getY();

                // following values may not be needed 
                double targetYaw = target.getYaw();
                
                //var pitch = target.getPitch();
                // double area = target.getArea();
                //double skew = target.getSkew();

                // auto to run l4 
                if (iAutoStage==FRONT_CAMERA){
                    driveSub.addCommands(
                    // init
                    new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPosition(0)),
                    new InstantCommand(() -> ArmSubsystem.getInstance().setArmEncoderPosition(0)),
                    // run to branch and l4
                    new ParallelDeadlineGroup(
                    new WaitCommand(1),  
                    // test with this line first -> then add rotation
                    new InstantCommand(()-> swerveSub.drive(new Translation2d(targetX, targetY), 0, false, true)),
                    // !! once above simple one works, modify and replace with the line below
                    //new InstantCommand(()-> swerveSub.drive(new Translation2d(targetX, targetY+HALF_BRANCH_DIFF*iAdjust), targetYaw, false, true)),
                    new ArmevatorToPosition(4)),
                    // dropping coral 
                    new ParallelDeadlineGroup(
                    new WaitCommand(1),
                    new ReverseCoralIntake(),
                    new RunElevator(true)
                    ),
                    
                    // backing up + setting for coral pickup 
                    new ParallelRaceGroup(
                    new WaitCommand(0.5),
                    new InstantCommand(() -> swerveSub.drive(new Translation2d(-0.3,-0.2), 0, false, true)),
                    new ArmevatorToPosition(0)
                    )
     
                    );
                }
                else if (iAutoStage == BACK_CAMERA) {
                    driveSub.addCommands(
                    // init
                    new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPosition(0)),
                    new InstantCommand(() -> ArmSubsystem.getInstance().setArmEncoderPosition(0)),
                    new InstantCommand(() -> CoralSubsystem.getInstance()),
                    // going to coral pickup
                    //new InstantCommand(() -> swerveSub.drive(new Translation2d(0.3,0), 0, false, true)),
                    new ParallelRaceGroup(
                    new WaitCommand(1),
                    new ArmevatorToPosition(1),
                    new InstantCommand(() -> swerveSub.drive(new Translation2d(targetX, targetY), 0, false, true)),
                    // wait response for coral before going to the next branch 
                    new InstantCommand(() -> coralSub.runIntake(0.5))
                                        )
                    );
                }
                System.out.println("AprilTagAuto End of process");
                
          }

      
   
     
    }
    }
    