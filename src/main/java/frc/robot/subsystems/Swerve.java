package frc.robot.subsystems;

import com.ctre.phoenix.sensors.Pigeon2;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Swerve extends SubsystemBase {
  private final Pigeon2 gyro;

  private SwerveDriveOdometry swerveOdometry;
  private SwerveModule[] swerveModules;

  private Field2d field;

  private final PIDController xController = new PIDController(Constants.AutoConstants.kPXController, 0, 0);
  private final PIDController yController = new PIDController(Constants.AutoConstants.kPYController, 0, 0);
  private final ProfiledPIDController profiledThetaController = new ProfiledPIDController(
    Constants.AutoConstants.kPThetaController, 0, 0,
    Constants.AutoConstants.kThetaControllerConstraints);
  private final PIDController thetaController = new PIDController(profiledThetaController.getP(), profiledThetaController.getI(), profiledThetaController.getD());

  public Swerve() {
    gyro = new Pigeon2(Constants.SwerveConstants.pigeonID);
    gyro.configFactoryDefault();
    zeroGyro();

    swerveOdometry = new SwerveDriveOdometry(Constants.SwerveConstants.swerveKinematics, getYaw(), getPositions());

    swerveModules =
        new SwerveModule[] {
          new SwerveModule(0, Constants.SwerveConstants.Mod0.constants),
          new SwerveModule(1, Constants.SwerveConstants.Mod1.constants),
          new SwerveModule(2, Constants.SwerveConstants.Mod2.constants),
          new SwerveModule(3, Constants.SwerveConstants.Mod3.constants)
        };

    field = new Field2d();
    SmartDashboard.putData("Field", field);
  }

  public void drive(
      Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
    SwerveModuleState[] swerveModuleStates =
        Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(
            fieldRelative
                ? ChassisSpeeds.fromFieldRelativeSpeeds(
                    translation.getX(), translation.getY(), rotation, getYaw())
                : new ChassisSpeeds(translation.getX(), translation.getY(), rotation));
    SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(swerveModuleStates[mod.moduleNumber], isOpenLoop);
    }
  }

  // This will be used in Auto, DON'T REMOVE
  public void setModuleStates(SwerveModuleState[] desiredStates) {
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.SwerveConstants.maxSpeed);

    for (SwerveModule mod : swerveModules) {
      mod.setDesiredState(desiredStates[mod.moduleNumber], false);
    }
  }

  public void setModuleStates(SwerveModuleState[] desiredStates, boolean openLoop, boolean steerInPlace){
    SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, Constants.AutoConstants.kMaxLinearSpeed);
    for(int i=0;i<4;i++){
      swerveModules[i].setDesiredState(desiredStates[i], openLoop);
    }
}

  public void setChassisSpeeds(ChassisSpeeds targetChassisSpeeds, boolean openLoop, boolean steerInPlace){
    setModuleStates(Constants.SwerveConstants.swerveKinematics.toSwerveModuleStates(targetChassisSpeeds), openLoop, steerInPlace);
}
public void stop(){
    drive(new Translation2d(0, 0), 0, false, true);
}

  public Pose2d getPose() {
    return swerveOdometry.getPoseMeters();
  }

  public SwerveModulePosition[] getPositions() {
    SwerveModulePosition currentStates[] = new SwerveModulePosition[swerveModules.length];
    for (int i = 0; i < swerveModules.length; i++) currentStates[i] = swerveModules[i].getPosition();
    return currentStates;
  }

  public void resetOdometry(Pose2d pose) {
   swerveOdometry.resetPosition(getYaw(), getPositions(), pose);
  }

  public SwerveModuleState[] getStates() {
    SwerveModuleState[] states = new SwerveModuleState[4];
    for (SwerveModule mod : swerveModules) states[mod.moduleNumber] = mod.getState();
    return states;
  }

  public void zeroGyro() {
    gyro.setYaw(0);
  }

  public Rotation2d getYaw() {
    return (Constants.SwerveConstants.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getYaw()): Rotation2d.fromDegrees(gyro.getYaw());
  }

  public PIDController getXController() {
    return xController;
  }
  public PIDController getYController() {
    return yController;
  }
  public PIDController getRotController() {
    return thetaController;
  }

  @Override
  public void periodic() {
    swerveOdometry.update(getYaw(), getPositions());
    field.setRobotPose(getPose());
  }
}