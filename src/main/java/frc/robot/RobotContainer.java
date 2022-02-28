package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.common.OCXboxController;
import frc.robot.subsystems.Drivetrain;

public class RobotContainer {

    private Drivetrain drivetrain = new Drivetrain();

    private OCXboxController controller = new OCXboxController(0);

    public RobotContainer(){
        configureBindings();
    }

    private void configureBindings(){
        
    }
}
