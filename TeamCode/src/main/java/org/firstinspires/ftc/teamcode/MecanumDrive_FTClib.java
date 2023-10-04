package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.drivebase.MecanumDrive;
import com.arcrobotics.ftclib.gamepad.GamepadEx;
import com.arcrobotics.ftclib.hardware.RevIMU;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.teamcode.subsystems.RobotHardware;

@TeleOp(name = "MecanumDrive_FTClib")
public class MecanumDrive_FTClib extends LinearOpMode {

    private Motor frontRight, backRight, frontLeft, backLeft;
    private MecanumDrive mecanum;
    private IMU imu;
    private IMU.Parameters imuParameters;

    @Override
    public void runOpMode() {
        // Declare our motors
        // Make sure your ID's match your configuration
        frontRight = new Motor(hardwareMap,"front_right_motor", Motor.GoBILDA.RPM_312);
        backRight  = new Motor(hardwareMap,"rear_right_motor",  Motor.GoBILDA.RPM_312);
        frontLeft  = new Motor(hardwareMap,"front_left_motor",  Motor.GoBILDA.RPM_312);
        backLeft   = new Motor(hardwareMap,"rear_left_motor", Motor.GoBILDA.RPM_312);

        mecanum = new MecanumDrive(frontLeft, frontRight, backLeft, backRight);

        // Retrieve the IMU from the hardware map
        imu = hardwareMap.get(IMU.class,"imu");
        // ***Adjust the orientation parameters to match your robot***
        imuParameters = new IMU.Parameters(new RevHubOrientationOnRobot(
                RevHubOrientationOnRobot.LogoFacingDirection.UP,
                RevHubOrientationOnRobot.UsbFacingDirection.FORWARD));
        // Without this, the REV Hub's orientation is assumed to be logo up / USB forward
        imu.initialize(imuParameters);

        waitForStart();

        while (opModeIsActive()) {

            double drive  = gamepad1.left_stick_x;
            double strafe = gamepad1.left_stick_y;
            double rotate = gamepad1.right_stick_y;

            mecanum.driveRobotCentric(strafe, drive, rotate);
        }
    }
}
