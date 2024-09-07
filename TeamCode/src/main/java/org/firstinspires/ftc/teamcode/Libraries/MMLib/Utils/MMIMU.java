package org.firstinspires.ftc.teamcode.Libraries.MMLib.Utils;

import com.qualcomm.hardware.bosch.BHI260IMU;
import com.qualcomm.hardware.bosch.BNO055IMUNew;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.MMRobot;
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
 * just with the .class after. for example:
 * <p>
 * MMIMU<‎BHI260IMU> imu = new MMIMU<>(BHI260IMU.class);
 * @param <T>
 */
public class MMIMU<T extends I2cDeviceSynchDevice & IMU> {

    private final T imu;

    public MMIMU(Class<T> imuClass) {

        if(imuClass == BNO055IMUNew.class) {

            imu = imuClass.cast(MMRobot.getInstance().mmSystems.hardwareMap.get(BNO055IMUNew.class, Configuration.IMU));

        } else if(imuClass == BHI260IMU.class) {

            imu = imuClass.cast(MMRobot.getInstance().mmSystems.hardwareMap.get(BHI260IMU.class, Configuration.IMU));

        } else throw new IllegalArgumentException("IMU must be BNO055IMUNew or BHI260IMU");


        assert imu != null;
        imu.initialize();
    }

    public T getImu() {
        return imu;
    }

    public YawPitchRollAngles getRobotYawPitchRollAngles() {
        return imu.getRobotYawPitchRollAngles();
    }

    public double getYawInDegrees() {
        return imu.getRobotYawPitchRollAngles().getYaw(AngleUnit.DEGREES);
    }

    public double getRollInDegrees() {
        return imu.getRobotYawPitchRollAngles().getRoll(AngleUnit.DEGREES);
    }

    public double getPitchInDegrees() {
        return imu.getRobotYawPitchRollAngles().getPitch(AngleUnit.DEGREES);
    }


}
