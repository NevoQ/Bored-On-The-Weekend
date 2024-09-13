package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import androidx.annotation.NonNull;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

/**
 * this class should help u reach the imu easily.
 * notice that there are 2 types of IMUs out there.
 * the old ones were always the {@link BNO055IMUNew} type.
 * but in the start of IntoTheDeep2025 season, we found out that the new control hub,
 * comes with a new type of IMU, the new type was {@link BHI260IMU}.
 * this is why this class was created.
 * in order to choose which type of IMU u would like to use,
 * insert the {@link BNO055IMUNew} or {@link BHI260IMU} type in the generics section of the variable.
 * for example:
 * <p>
 * MMIMU<‎BHI260IMU> imu
 *<p>
 * in the creation of the IMU in {@link org.firstinspires.ftc.teamcode.MMSystems MMSystems} u should insert the same type,
 * just with the .class after, and then the orientation of the control hub. for example:
 * <p>
 * MMIMU<‎BHI260IMU> imu = new MMIMU<>(
 *                  BHI260IMU.class,
 *                  hardwareMap,
 *                  RevHubOrientationOnRobot.LogoFacingDirection.RIGHT,
 *                  RevHubOrientationOnRobot.UsbFacingDirection.UP
 * );
 * @param <T>
 */
public class MMIMU<T extends I2cDeviceSynchDevice & IMU> {

    private final T imu;

    /**
     * this constructor is used in order to create the imu object.
     * <p>
     * first u should specify which type of imu you're using.
     * next, u should insert the orientation of the control hub.
     * @param imuClass the imu type class.
     * @param hardwareMap the hardwareMap object.
     * @param logoFacingDirection the logo facing direction, use {@link com.qualcomm.hardware.rev.RevHubOrientationOnRobot.LogoFacingDirection LogoFacingDirection}.
     * @param usbFacingDirection the usb facing direction, use {@link com.qualcomm.hardware.rev.RevHubOrientationOnRobot.UsbFacingDirection UsbFacingDirection}.
     */
    public MMIMU(
            Class<T> imuClass,
            HardwareMap hardwareMap,
            @NonNull RevHubOrientationOnRobot.LogoFacingDirection logoFacingDirection,
            @NonNull RevHubOrientationOnRobot.UsbFacingDirection usbFacingDirection
    ) {

        if(imuClass == BNO055IMUNew.class) {

            imu = imuClass.cast(hardwareMap.get(BNO055IMUNew.class, Configuration.IMU));

        } else if(imuClass == BHI260IMU.class) {

            imu = imuClass.cast(hardwareMap.get(BHI260IMU.class, Configuration.IMU));

        } else throw new IllegalArgumentException("IMU must be BNO055IMUNew or BHI260IMU");

        IMU.Parameters parameters = new IMU.Parameters(
                new RevHubOrientationOnRobot(
                        logoFacingDirection,
                        usbFacingDirection
                )
        );

        assert imu != null;
        imu.initialize(parameters);
    }

    /**
     * this method returns the imu object
     * @return imu object
     */
    public T getImu() {
        return imu;
    }

    /**
     * this is used to get the yaw, pitch and roll (as one object).
     * @return the {@link YawPitchRollAngles} object
     */
    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    /**
     * @return the robot's yaw in degrees.
     */
    public double getYawInDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    /**
     * @return the robot's roll in degrees.
     */
    public double getRollInDegrees() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    /**
     * @return the robot's pitch in degrees.
     */
    public double getPitchInDegrees() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }

    /**
     * this method is used to reset the robot's yaw.
     */
    public void resetYaw() {
        imu.resetYaw();
    }


}
