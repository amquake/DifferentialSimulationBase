// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXSimCollection;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.BasePigeonSimCollection;
import com.ctre.phoenix.sensors.WPI_Pigeon2;

import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.drive.DifferentialDrive.WheelSpeeds;
import edu.wpi.first.wpilibj.simulation.BatterySim;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.simulation.RoboRioSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.TalonUtil;

public class Drivetrain extends SubsystemBase {
    
    private final WPI_TalonFX leftMotor = new WPI_TalonFX(0);
    private final WPI_TalonFX rightMotor = new WPI_TalonFX(1);

    private final WPI_Pigeon2 gyro = new WPI_Pigeon2(0);

    private final double kGearRatio = 8.8888; // Motor rotations per wheel rotations
    private final double kWheelDiameterMeters = Units.inchesToMeters(6);
    private final double kWheelCircumferenceMeters = kWheelDiameterMeters * Math.PI;
    private final double kTrackWidthMeters = Units.inchesToMeters(23); // distance between left/right wheels

    private final SimpleMotorFeedforward linearFF = new SimpleMotorFeedforward(0.23, 2.3, 0.32); // Feedforward values

    private final DifferentialDriveKinematics kinematics = new DifferentialDriveKinematics(kTrackWidthMeters);
    private final DifferentialDriveOdometry odometry = new DifferentialDriveOdometry(gyro.getRotation2d());
    private final Field2d field2d = new Field2d();
    
    /** Creates a new Drivetrain. */
    public Drivetrain() {
        SmartDashboard.putData(field2d);
    }
    
    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        odometry.update(
            gyro.getRotation2d(),
            TalonUtil.positionToMeters(
                leftMotor.getSelectedSensorPosition(),
                kGearRatio, kWheelCircumferenceMeters
            ),
            TalonUtil.positionToMeters(
                rightMotor.getSelectedSensorPosition(),
                kGearRatio, kWheelCircumferenceMeters
            )
        );
        field2d.setRobotPose(getPose2d());
    }

    public DifferentialDriveWheelSpeeds getWheelSpeeds(){
        return new DifferentialDriveWheelSpeeds(
            TalonUtil.velocityToMeters(
                leftMotor.getSelectedSensorVelocity(),
                kGearRatio, kWheelCircumferenceMeters
            ),
            TalonUtil.velocityToMeters(
                rightMotor.getSelectedSensorVelocity(),
                kGearRatio, kWheelCircumferenceMeters
            )
        );
    }
    public ChassisSpeeds getChassisSpeeds(){
        return kinematics.toChassisSpeeds(getWheelSpeeds());
    }
    public Pose2d getPose2d(){
        return odometry.getPoseMeters();
    }

    //----- 


    //----- Simulation logic
    private final SimpleMotorFeedforward angularFF = new SimpleMotorFeedforward(0.261, 2.4, 0.49);
    private BasePigeonSimCollection gyroSim = gyro.getSimCollection();
    private TalonFXSimCollection leftMotorSim = leftMotor.getSimCollection();
    private TalonFXSimCollection rightMotorSim = rightMotor.getSimCollection();
    private DifferentialDrivetrainSim drivetrainSim = new DifferentialDrivetrainSim(
        LinearSystemId.identifyDrivetrainSystem(linearFF.kv, linearFF.ka, angularFF.kv, angularFF.ka),
        DCMotor.getNEO(2),
        kGearRatio,
        kTrackWidthMeters,
        kWheelDiameterMeters,
        VecBuilder.fill(
            0.005, 0.005, 0.0001,
            0.05, 0.05,
            0, 0
        )
    );

    @Override
    public void simulationPeriodic(){
        double leftVoltage = leftMotor.getMotorOutputVoltage();
        double rightVoltage = rightMotor.getMotorOutputVoltage();
        drivetrainSim.setInputs(leftVoltage, rightVoltage);
        drivetrainSim.update(0.02);

        double leftVelocityNative = TalonUtil.metersToVelocity(
            drivetrainSim.getLeftVelocityMetersPerSecond(),
            kGearRatio, kWheelCircumferenceMeters
        );
        leftMotorSim.setIntegratedSensorVelocity((int)leftVelocityNative);
        double leftPositionNative = TalonUtil.metersToPosition(
            drivetrainSim.getLeftPositionMeters(),
            kGearRatio, kWheelCircumferenceMeters
        );
        leftMotorSim.setIntegratedSensorRawPosition((int)leftPositionNative);
        leftMotorSim.setSupplyCurrent(drivetrainSim.getLeftCurrentDrawAmps());

        double rightVelocityNative = TalonUtil.metersToVelocity(
            drivetrainSim.getRightVelocityMetersPerSecond(),
            kGearRatio, kWheelCircumferenceMeters
        );
        rightMotorSim.setIntegratedSensorVelocity((int)rightVelocityNative);
        double rightPositionNative = TalonUtil.metersToPosition(
            drivetrainSim.getRightPositionMeters(),
            kGearRatio, kWheelCircumferenceMeters
        );
        rightMotorSim.setIntegratedSensorRawPosition((int)rightPositionNative);
        rightMotorSim.setSupplyCurrent(drivetrainSim.getRightCurrentDrawAmps());

        gyroSim.setRawHeading(drivetrainSim.getHeading().getDegrees());

        RoboRioSim.setVInVoltage(BatterySim.calculateDefaultBatteryLoadedVoltage(
            drivetrainSim.getLeftCurrentDrawAmps()
        ));
        leftMotorSim.setBusVoltage(RobotController.getBatteryVoltage());
    }
}
