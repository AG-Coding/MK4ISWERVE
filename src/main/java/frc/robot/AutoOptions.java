package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.commands.PPSwerveFollower;
import frc.robot.subsystems.Swerve;

public class AutoOptions {
  private SendableChooser<CommandBase> autoOptions = new SendableChooser<>();
  private final Swerve drive;

  public AutoOptions(Swerve drive) {
    this.drive = drive;
    autoOptions.setDefaultOption("Straight Path", new PPSwerveFollower(drive, "Straight Path", Constants.AutoConstants.kMediumPathConstraints, true));
    submit();
  }

  public CommandBase getAutoCommand() {
    var cmd = autoOptions.getSelected();
    if (cmd == null) {
      cmd = Commands.none();
    }
    return cmd;
  }

  public void submit() {
    SmartDashboard.putData("Auto Options", autoOptions);
  }
}