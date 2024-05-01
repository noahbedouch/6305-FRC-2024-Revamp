package frc.robot.subsystems;

import com.ctre.phoenix6.configs.Pigeon2Configuration;
import com.ctre.phoenix6.hardware.Pigeon2;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import frc.lib.swerve.SwerveModule;
import frc.robot.Constants;

public class Drive {
    Translation2d mfrontLeftLocation = new Translation2d(0.737, 0.737);
    Translation2d mfrontRightLocation = new Translation2d(0.737, -0.737);
    Translation2d mbackLeftLocation = new Translation2d(-0.737, 0.737);
    Translation2d mbackRightLocation = new Translation2d(-0.737, -0.737);

    SwerveDriveKinematics kinematics = new SwerveDriveKinematics(mfrontLeftLocation, mfrontRightLocation, mbackLeftLocation, mbackRightLocation);

    private final Pigeon2 pigeon = new Pigeon2(Constants.pigeonID, Constants.canivoreName);

    SwerveModule[] modules;
    public static final int frontLeftID = 0;
    public static final int frontRightID = 1;
    public static final int backLeftID = 2;
    public static final int backRightID = 3;

    private Drive() {
        modules = new SwerveModule[4];
        modules[frontLeftID] = new SwerveModule(
            Constants.frontLeftDrive,
            Constants.frontLeftAngle,
            Constants.frontLeftEncoder,
            Constants.frontLeftOffset
        );
        modules[frontRightID] = new SwerveModule(
            Constants.frontRightDrive,
            Constants.frontRightAngle,
            Constants.frontRightEncoder,
            Constants.frontRightOffset
        );
        modules[backLeftID] = new SwerveModule(
            Constants.backLeftDrive,
            Constants.backLeftAngle,
            Constants.backLeftEncoder,
            Constants.backLeftOffset
        );
        modules[backRightID] = new SwerveModule(
            Constants.backRightDrive,
            Constants.backRightAngle,
            Constants.backRightEncoder,
            Constants.backRightOffset
        );
    }
}
