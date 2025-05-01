package frc.robot.commands;


    import org.photonvision.PhotonCamera;
    import org.photonvision.targeting.PhotonPipelineResult;
    import org.photonvision.targeting.PhotonTrackedTarget;

    import edu.wpi.first.math.geometry.Translation2d;
    import edu.wpi.first.wpilibj.Timer;
    import edu.wpi.first.wpilibj2.command.Command;
    import edu.wpi.first.wpilibj2.command.InstantCommand;
    import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
    import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
    import edu.wpi.first.wpilibj2.command.WaitCommand;
    import frc.robot.commands.Swerve.AutoAlignAprilTags;
    import frc.robot.commands.Swerve.SwerveCommand;
    import frc.robot.commands.Armevator.ArmevatorToPosition;
    import frc.robot.commands.Armevator.RunElevator;
    import frc.robot.commands.Coral.ReverseCoralIntake;
    import frc.robot.subsystems.ArmSubsystem;
    import frc.robot.subsystems.CoralSubsystem;
    import frc.robot.subsystems.ElevatorSubsystem;
    import frc.robot.subsystems.SwerveSubsystem;
    import frc.robot.subsystems.VisionSubsystem;
    import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
    
    public class AprilTagAuto extends Command{
  
        // !! this value needs to be checked and updated
        private static final double HALF_BRANCH_DIFF = 0.1;     
         
         // camera name
        private static final boolean FRONT_CAMERA = true; 
        private static final boolean BACK_CAMERA = false; 

        private final Timer timer = new Timer();
        boolean autoFinished = false;
        
        // robotics path for AprilTag
        // 1- to AprilTag branch; 0- to ArpilTag Coral station
        boolean bFront = FRONT_CAMERA;
        // branch: -1- left, 1-right
        boolean bLeftBranch = true;


        private PhotonCamera camera;
        //private SwerveSubsystem swerveSub ;
        private VisionSubsystem visionSub ;
    
        // private final DriveForwardAuto driveSub = new DriveForwardAuto();
        // private final CoralSubsystem coralSub = new CoralSubsystem();
        private final SequentialCommandGroup sequentialSub = new SequentialCommandGroup();
        
        
            // Called when the command is initially scheduled.
        @Override
        public void initialize() {
            timer.reset();
            timer.start();

            //this.swerveSub = SwerveSubsystem.getInstance();
            this.visionSub = VisionSubsystem.getInstance();

            if (bFront == FRONT_CAMERA) {
                this.camera = visionSub.frontAprilCamera;
                System.out.println("1-link to front camera");
            } else if (bFront == BACK_CAMERA) {
                this.camera = visionSub.backAprilCamera;
                System.out.println("2-link to front camera");
            } else {
                //this.camera = visionSub.frontAprilCamera;
                System.out.println("3-link to front camera");
            }
  }

        // Called every time the scheduler runs while the command is scheduled.
        @Override
        public void execute() {
            // Vanessa
            while (timer.get()<15 && autoFinished == false)  {
                System.out.println("time-"+timer.get());

                processAutoAccess(bFront, bLeftBranch);
        
                // switch camera after processing
                if (bFront==FRONT_CAMERA) {
                bFront = BACK_CAMERA;   
                }
                else if (bFront==BACK_CAMERA) { 
                bFront = FRONT_CAMERA;
                bLeftBranch = bLeftBranch ? false : true;
                }
            
                // Add a delay to avoid busy-waiting
                try {
                    Thread.sleep(100);
                } catch (InterruptedException e) {
                    autoFinished = true;
                    System.out.println("autonomousPeriodic error");
                }
                // turn moter off
                autoFinished = true;
                // stop moters...
            
            }
        }
    
        // bFront:1/2-to branch/coral pickup; iBranch: 1/-1-left or right for coral drop
        public void processAutoAccess(boolean bFront, boolean bBranch) {
            PhotonPipelineResult result = camera.getLatestResult();
            if (result.hasTargets()) {
                // PhotonTrackedTarget target = result.getBestTarget();
                // double targetX = target.getBestCameraToTarget().getX();
                // double targetY = target.getBestCameraToTarget().getY();
                // double targetYaw = target.getYaw();
                
                // //var pitch = target.getPitch();
                // // double area = target.getArea();
                // //double skew = target.getSkew();

                // auto to run l4 
                if (bFront==FRONT_CAMERA){

                    sequentialSub.addCommands (
                    new ParallelDeadlineGroup(
                        new WaitCommand(1.5), 
                        new ArmevatorToPosition(4)
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(1),
                        new SwerveCommand(
                        () -> -0.15,
                        () -> 0, 
                        () -> 0, 
                        () -> true)
                    ),
                    new AutoAlignAprilTags(bFront, bLeftBranch),
                    new ParallelDeadlineGroup(
                        new WaitCommand(1), 
                        new ReverseCoralIntake()
                    ),
                    new ParallelRaceGroup(
                        new WaitCommand(1.0),
                        new SwerveCommand(
                        () -> 0.15,
                        () -> 0, 
                        () -> 0,  
                        () -> true)
                    ),
                    new ParallelDeadlineGroup(
                        new WaitCommand(1.5), 
                        new ArmevatorToPosition(0)
                    )
                    );


                    // driveSub.addCommands(
                    // // init
                    // new InstantCommand(() -> ElevatorSubsystem.getInstance().setElevatorPosition(0)),
                    // new InstantCommand(() -> ArmSubsystem.getInstance().setArmEncoderPosition(0)),
                    // // run to branch and l4
                    // new ParallelDeadlineGroup(
                    // new WaitCommand(1),  
                    // // test with this line first -> then add rotation
                    // new InstantCommand(()-> swerveSub.drive(new Translation2d(targetX, targetY), 0, false, true)),
                    // // !! once above simple one works, modify and replace with the line below
                    // //new InstantCommand(()-> swerveSub.drive(new Translation2d(targetX, targetY+HALF_BRANCH_DIFF*iAdjust), targetYaw, false, true)),
                    // new ArmevatorToPosition(4)),
                    // // dropping coral 
                    // new ParallelDeadlineGroup(
                    // new WaitCommand(1),
                    // new ReverseCoralIntake(),
                    // new RunElevator(true)
                    // ),
                    
                    // // backing up + setting for coral pickup 
                    // new ParallelRaceGroup(
                    // new WaitCommand(0.5),
                    // new InstantCommand(() -> swerveSub.drive(new Translation2d(-0.3,-0.2), 0, false, true)),
                    // new ArmevatorToPosition(0)
                    // )
     
                    // );
                }
                else if (bFront == BACK_CAMERA) {
                    sequentialSub.addCommands (
                        new ParallelDeadlineGroup(
                            new WaitCommand(1.5), 
                            new ArmevatorToPosition(0)
                        ),
                        new ParallelRaceGroup(
                            new WaitCommand(1),
                            new SwerveCommand(
                            () -> -0.15,
                            () -> 0, 
                            () -> 0, 
                            () -> true)
                        ),
                        new AutoAlignAprilTags(bFront, bLeftBranch),
                        new ParallelDeadlineGroup(
                            new WaitCommand(1), 
                            new ReverseCoralIntake()
                        ),
                        new ParallelRaceGroup(
                            new WaitCommand(1.0),
                            new SwerveCommand(
                            () -> 0.15,
                            () -> 0, 
                            () -> 0,  
                            () -> true)
                        ),
                        new ParallelDeadlineGroup(
                            new WaitCommand(1.5), 
                            new ArmevatorToPosition(0)
                        )
                        );

                System.out.println("AprilTagAuto End of process");
                
                  }

                }
        
            
            }
}
    
    