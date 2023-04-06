package frc.robot.commands;

import java.util.function.Supplier;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.subsystems.Swerve;

public class PPSwerveFollower extends CommandBase {
    
    private final Swerve drive;
    private PathPlannerTrajectory path;
    private Supplier<PathPlannerTrajectory> pathSupplier;
    private final boolean resetOdom;

    private CommandBase controllerCommand = Commands.none();
    private boolean initialized = false;

    public PPSwerveFollower(
            Swerve drive, String pathName,
            PathConstraints constraints, boolean resetOdom) {
        this.drive = drive;
        path = PathPlanner.loadPath(pathName, constraints);
        pathSupplier = null;
        this.resetOdom = resetOdom;

        addRequirements(drive);
    }
    public PPSwerveFollower(
            Swerve drive, Supplier<PathPlannerTrajectory> pathSupplier,
            boolean resetOdom) {
        this.drive = drive;
        this.path = null;
        this.pathSupplier = pathSupplier;
        this.resetOdom = resetOdom;

        addRequirements(drive);
    }
    
    @Override
    public void initialize() {
        if(pathSupplier != null) {
            path = pathSupplier.get();
        }
        if(path == null) {
            end(false);
            DriverStation.reportError("FOllowing a null Path!", true);
            return;
        }
        if(pathSupplier == null && !initialized) {
            path = PathPlannerTrajectory.transformTrajectoryForAlliance(
                path,
                DriverStation.getAlliance());
        }

        if(resetOdom) drive.resetOdometry(path.getInitialHolonomicPose());

        controllerCommand = new PPSwerveControllerCommand(
            path,
            drive::getPose,
            drive.getXController(),
            drive.getYController(),
            drive.getRotController(),
            (chassisSpeeds) -> drive.setChassisSpeeds(chassisSpeeds, false, true),
            false,
            drive
        );
        controllerCommand.initialize();
        initialized = true;
    }
    
    @Override
    public void execute() {
        controllerCommand.execute();
    }
    
    @Override
    public void end(boolean interrupted) {
        controllerCommand.end(interrupted);
    }
    
    @Override
    public boolean isFinished() {
        return controllerCommand.isFinished();
    }
}