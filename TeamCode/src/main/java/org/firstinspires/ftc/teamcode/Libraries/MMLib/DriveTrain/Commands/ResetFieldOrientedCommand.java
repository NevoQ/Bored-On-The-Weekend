package org.firstinspires.ftc.teamcode.Libraries.MMLib.DriveTrain.Commands;

import com.arcrobotics.ftclib.command.InstantCommand;

import org.firstinspires.ftc.teamcode.MMRobot;

/**
 * this command resets the imu's yaw.
 */
public class ResetFieldOrientedCommand extends InstantCommand {
    public ResetFieldOrientedCommand() {
        super(
                () -> MMRobot.getInstance().mmSystems.imu.resetYaw()
        );
    }
}
