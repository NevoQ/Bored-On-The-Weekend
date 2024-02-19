package org.firstinspires.ftc.teamcode.Autonomous;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.RobotControl;
import org.firstinspires.ftc.teamcode.Utils.AllianceColor;
import org.firstinspires.ftc.teamcode.Utils.Side;

@Autonomous(name = "AutonomousRightRed")
@Disabled
public class AutonomousRightRed extends CommandOpMode {

    RobotControl robot;
    @Override
    public void initialize() {
        robot = new RobotControl(RobotControl.OpModeType.AUTO, AllianceColor.RED, Side.RIGHT, hardwareMap ,gamepad1,gamepad2,telemetry);
    }

    @Override
    public void run() {
        super.run();
    }
}