package frc.robot.subsystems;

public class SubsystemsInst {
    public Drivetrain drivetrain;
    public Vision vision;
    public ArmSubsystem armSubsystem;
   
   
    private static SubsystemsInst inst;

    private SubsystemsInst() {
        drivetrain = new Drivetrain();
        vision = new Vision();
        armSubsystem = new ArmSubsystem();

    }

    public static SubsystemsInst getInst () {
        if(inst == null) inst = new SubsystemsInst();

        return inst;

    }
    
}