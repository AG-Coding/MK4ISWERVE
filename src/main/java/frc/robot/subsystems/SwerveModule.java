package frc.robot.subsystems;

import com.ctre.phoenix.sensors.CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import frc.lib.Configurations.CTREConfigs;
import frc.lib.Util.CANCoderUtil;
import frc.lib.Util.CANSparkMaxUtil;
import frc.lib.Util.CANCoderUtil.CCUsage;
import frc.lib.Util.CANSparkMaxUtil.Usage;
import frc.lib.math.OnboardModuleState;
import frc.robot.Constants;

public class SwerveModule {
  public int moduleNumber;
  private Rotation2d lastAngle;
  private Rotation2d angleOffset;

  private CANSparkMax angleMotor;
  private CANSparkMax driveMotor;

  private RelativeEncoder driveEncoder;
  private RelativeEncoder integratedAngleEncoder;
  private CANCoder angleEncoder;

  private final SparkMaxPIDController driveController;
  private final SparkMaxPIDController angleController;

  private final SimpleMotorFeedforward feedforward =
      new SimpleMotorFeedforward(
          Constants.SwerveConstants.driveKS, Constants.SwerveConstants.driveKV, Constants.SwerveConstants.driveKA);

  public SwerveModule(int moduleNumber, frc.lib.Configurations.SwerveModuleConstants moduleConstants) {
    this.moduleNumber = moduleNumber;
    angleOffset = moduleConstants.angleOffset;

    // Angle Encoder Config
    angleEncoder = new CANCoder(moduleConstants.cancoderID);
    configAngleEncoder();

    // Angle Motor Config
    angleMotor = new CANSparkMax(moduleConstants.angleMotorID, MotorType.kBrushless);
    integratedAngleEncoder = angleMotor.getEncoder();
    angleController = angleMotor.getPIDController();
    configAngleMotor();

    // Drive Motor Config 
    driveMotor = new CANSparkMax(moduleConstants.driveMotorID, MotorType.kBrushless);
    driveEncoder = driveMotor.getEncoder();
    driveController = driveMotor.getPIDController();
    configDriveMotor();

    lastAngle = getState().angle;
  }

  public void setDesiredState(SwerveModuleState desiredState, boolean isOpenLoop) {
    // Custom optimize command, since default WPILib optimize assumes continuous controller which REV and CTRE are not
    desiredState = OnboardModuleState.optimize(desiredState, getState().angle);
    setAngle(desiredState);
    setSpeed(desiredState, isOpenLoop);
  }

  private void resetToAbsolute() {
    double absolutePosition = getCanCoder().getDegrees() - angleOffset.getDegrees();
    integratedAngleEncoder.setPosition(absolutePosition);
  }

  private void configAngleEncoder() {
    angleEncoder.configFactoryDefault();
    CANCoderUtil.setCANCoderBusUsage(angleEncoder, CCUsage.kMinimal);
    angleEncoder.configAllSettings(CTREConfigs.swerveCanCoderConfig);
  }

  private void configAngleMotor() {
    angleMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(angleMotor, Usage.kPositionOnly);
    angleMotor.setSmartCurrentLimit(Constants.SwerveConstants.angleContinuousCurrentLimit);
    angleMotor.setInverted(Constants.SwerveConstants.angleInvert);
    angleMotor.setIdleMode(Constants.SwerveConstants.angleNeutralMode);
    integratedAngleEncoder.setPositionConversionFactor(Constants.SwerveConstants.angleConversionFactor);
    angleController.setP(Constants.SwerveConstants.angleKP);
    angleController.setI(Constants.SwerveConstants.angleKI);
    angleController.setD(Constants.SwerveConstants.angleKD);
    angleController.setFF(Constants.SwerveConstants.angleKFF);
    angleMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    angleMotor.burnFlash();
    resetToAbsolute();
  }

  private void configDriveMotor() {
    driveMotor.restoreFactoryDefaults();
    CANSparkMaxUtil.setCANSparkMaxBusUsage(driveMotor, Usage.kAll);
    driveMotor.setSmartCurrentLimit(Constants.SwerveConstants.driveContinuousCurrentLimit);
    driveMotor.setInverted(Constants.SwerveConstants.driveInvert);
    driveMotor.setIdleMode(Constants.SwerveConstants.driveNeutralMode);
    driveEncoder.setVelocityConversionFactor(Constants.SwerveConstants.driveConversionVelocityFactor);
    driveEncoder.setPositionConversionFactor(Constants.SwerveConstants.driveConversionPositionFactor);
    driveController.setP(Constants.SwerveConstants.angleKP);
    driveController.setI(Constants.SwerveConstants.angleKI);
    driveController.setD(Constants.SwerveConstants.angleKD);
    driveController.setFF(Constants.SwerveConstants.angleKFF);
    driveMotor.enableVoltageCompensation(Constants.SwerveConstants.voltageComp);
    driveMotor.burnFlash();
    driveEncoder.setPosition(0.0);
  }

  private void setSpeed(SwerveModuleState desiredState, boolean isOpenLoop) {
    if (isOpenLoop) {
      double percentOutput = desiredState.speedMetersPerSecond / Constants.SwerveConstants.maxSpeed;
      driveMotor.set(percentOutput);
    } else {
      driveController.setReference(
          desiredState.speedMetersPerSecond,
          ControlType.kVelocity,
          0,
          feedforward.calculate(desiredState.speedMetersPerSecond));
    }
  }

  private void setAngle(SwerveModuleState desiredState) {
    // Prevent rotating module if speed is less then 1%. Prevents jittering.
    Rotation2d angle =
        (Math.abs(desiredState.speedMetersPerSecond) <= (Constants.SwerveConstants.maxSpeed * 0.01))
            ? lastAngle
            : desiredState.angle;

    angleController.setReference(angle.getDegrees(), ControlType.kPosition);
    lastAngle = angle;
  }

  private Rotation2d getAngle() {
    return Rotation2d.fromDegrees(integratedAngleEncoder.getPosition());
  }

  public SwerveModulePosition getPosition() {
    double distance = driveEncoder.getPosition();
    Rotation2d rot = new Rotation2d(angleEncoder.getPosition());
    return new SwerveModulePosition(distance, rot);
  }

  public Rotation2d getCanCoder() {
    return Rotation2d.fromDegrees(angleEncoder.getAbsolutePosition());
  }

  public SwerveModuleState getState() {
    return new SwerveModuleState(driveEncoder.getVelocity(), getAngle());
  }
}