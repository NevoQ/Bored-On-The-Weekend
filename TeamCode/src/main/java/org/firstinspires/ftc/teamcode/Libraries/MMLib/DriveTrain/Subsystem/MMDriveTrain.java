package org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Subsystem;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.geometry.Vector2d;
import com.roboctopi.cuttlefish.utils.Direction;

import org.apache.commons.math3.linear.MatrixUtils;
import org.apache.commons.math3.linear.RealMatrix;
import org.apache.commons.math3.linear.RealVector;
import org.firstinspires.ftc.teamcode.Libraries.CuttlefishFTCBridge.src.devices.CuttleMotor;
import org.firstinspires.ftc.teamcode.MMRobot;
import org.firstinspires.ftc.teamcode.Utils.Configuration;

/**
 * this class is an example of a mecanum drive train.
 */
public class MMDriveTrain extends SubsystemBase {

    final double[][] transformationMatrix = {
            {1, 1, 1}, //frontLeft
            {-1, 1, 1}, //backLeft
            {-1, 1, -1}, //frontRight
            {1, 1, -1} //backRight
    };

    MMRobot mmRobot = MMRobot.getInstance();

    private final CuttleMotor motorFR, motorFL, motorBL, motorBR;

    public MMDriveTrain() {
        super(); //register this subsystem, in order to schedule default command later on.

        motorFL = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_FRONT_LEFT);
        motorBL = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_BACK_LEFT);
        motorFR = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_FRONT_RIGHT);
        motorBR = new CuttleMotor(mmRobot.mmSystems.controlHub, Configuration.DRIVE_TRAIN_BACK_RIGHT);

        //TODO: reverse motors as needed
        motorFL.setDirection(Direction.REVERSE);
        motorBL.setDirection(Direction.REVERSE);

    }

    /**
     * this method translates the joystick values into motor power
     * @param x the value on the x axis
     * @param y the value on the y axis
     * @param yaw the value on the yaw axis
     * @return the power array to feed the motors, by this order:
     * <p>
     * frontLeft
     * <p>
     * backLeft
     * <p>
     * frontRight
     * <p>
     * backRight
     */
    private double[] joystickToPower(double x, double y, double yaw) {

        //v = (x, y, yaw)^t (3x1)
        RealVector joystickVector = MatrixUtils.createRealVector(new double[] {
                x,
                y,
                yaw
        });

        RealMatrix matrixT = MatrixUtils.createRealMatrix(transformationMatrix); //4x3

        //calculation of the power needed by T constants
        RealVector powerVector = matrixT.operate(joystickVector); //p = Tv

        double[] powerArray = powerVector.toArray(); //4x1

        //normalize the array
        for(int i = 0; i < powerArray.length; i++) {
            powerArray[i] = powerArray[i] / Math.max(Math.abs(x) + Math.abs(y) + Math.abs(yaw), 1);
        }

        return powerArray;

    }

    /**
     * updates the power to the motors.
     * @param power the power array to feed to the motor.
     */
    private void setMotorPower(double[] power) {
        motorFL.setPower(power[0]);
        motorBL.setPower(power[1]);
        motorFR.setPower(power[2]);
        motorBR.setPower(power[3]);
    }

    /**
     * normal (non-field-oriented) arcade drive
     * @param x the value on the x axis
     * @param y the value on the y axis
     * @param yaw the value on the yaw axis
     */
    public void drive(double x, double y, double yaw) {
        setMotorPower(joystickToPower(x, y, yaw));
    }

    /**
     * this method turns the normal joystick values, from arcade drive, to field oriented drive.
     * @param x the value on the x axis
     * @param y the value on the y axis
     * @param yaw the value on the yaw axis
     */
    public void fieldOrientedDrive(double x, double y, double yaw) {
        Vector2d joystickDirection = new Vector2d(x, y);
        Vector2d fieldOrientedVector = joystickDirection.rotateBy(-mmRobot.mmSystems.imu.getYawInDegrees());
        drive(fieldOrientedVector.getX(), fieldOrientedVector.getY(), yaw);
    }

}