package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.LED;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.objects.OpmodeHeading;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu;
import org.firstinspires.ftc.teamcode.subsystems.HydraImu_navx;

import java.util.List;

@TeleOp(name = "HyDriveOnly")
public class HyDriveOnly extends LinearOpMode {
  private HydraImu mImu;
  private DcMotor MotDrFrLt;
  private DcMotor MotDrBkLt;
  private DcMotor MotDrFrRt;
  private DcMotor MotDrBkRt;
  double cTrgBtnThresh;
  int cDriveBoosted;
  double cDriveNormal;
  double cDriveSlow;
  boolean cFieldCentric;

  /**
   * This function is executed when this OpMode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    MotDrFrLt = hardwareMap.get(DcMotor.class, "MotDrFrLt");
    MotDrBkLt = hardwareMap.get(DcMotor.class, "MotDrBkLt");
    MotDrFrRt = hardwareMap.get(DcMotor.class, "MotDrFrRt");
    MotDrBkRt = hardwareMap.get(DcMotor.class, "MotDrBkRt");
    // Initialize Constant Variables
    // Trigger button deadband for any trigger press
    cTrgBtnThresh = 0.1;
    // Drive motor power level scaling [max 1]
    cDriveBoosted = 1;
    cDriveNormal = 0.9;
    cDriveSlow = 0.5;
    cFieldCentric = true;
    // Initialization Routines
    // Initialize the IMU with non-default settings. To use this block,
    // plug one of the "new IMU.Parameters" blocks into the parameters socket.
    // Create a Parameters object for use with an IMU in a REV Robotics Control Hub or
    // Expansion Hub, specifying the hub's orientation on the robot via the direction that
    // the REV Robotics logo is facing and the direction that the USB ports are facing.
    HydraOpMode opMode = new HydraOpMode(telemetry, hardwareMap, null, null);
    mImu = new HydraImu_navx(opMode);
    InitDrive();
    while (!mImu.Connected() || mImu.Calibrating()) {
      if (isStopRequested() || !opModeIsActive()) {
        break;
      }
    }
    mImu.SetYawOffset(OpmodeHeading.GetOffset());
    telemetry.addData("Auton Yaw", OpmodeHeading.GetOffset());
    telemetry.update();
    waitForStart();
    while (opModeIsActive()) {
      // System processes
      ProcessDrive();
      // Update telemetry once for all processes
      telemetry.update();
      sleep(20);
    }
  }

  /**
   * Describe this function...
   */
  private void InitDrive() {
    // Set motor directions. Right side motors are reversed
    MotDrFrLt.setDirection(DcMotor.Direction.FORWARD);
    MotDrBkLt.setDirection(DcMotor.Direction.FORWARD);
    MotDrFrRt.setDirection(DcMotor.Direction.REVERSE);
    MotDrBkRt.setDirection(DcMotor.Direction.REVERSE);
  }

  /**
   * Describe this function...
   */
  private void ProcessDrive() {
    double drive;
    double strafe;
    double rotate;
    double rotX;
    double rotY;
    double driveMaxPower;
    double sum;
    double max;
    double frontLeftPower;
    double rearLeftPower;
    double frontRightPower;
    double rearRightPower;
    // get the yaw input from the gyro
    double yaw = 0;
    if (!mImu.Connected()) {
      telemetry.addData("Yaw", "disconnected");
    } else if (mImu.Calibrating()) {
      telemetry.addData("Yaw", "cal");
    } else {
      yaw = mImu.GetYaw();
      telemetry.addData("Yaw", yaw);
    }
    // Get driver controller input
    drive = gamepad1.left_stick_y;
    strafe = -gamepad1.left_stick_x * 1.1;
    if (gamepad1.cross && cFieldCentric) {
      // snap to the nearest 90 deg
      double snapHeading = yaw;
      if (yaw >= -180 && yaw <= -135) {
        snapHeading = -180;
      } else if (yaw > -135 && yaw <= -45) {
        snapHeading = -90;
      } else if (yaw > -45 && yaw <= 45) {
        snapHeading = 0;
      } else if (yaw > 45 && yaw <= 135) {
        snapHeading = 90;
      } else if (yaw > 135 && yaw <= 180) {
        snapHeading = 180;
      }
      rotate = -Math.sin((yaw - snapHeading) * Math.PI / 180) * 1.1;
    } else {
      rotate = -gamepad1.right_stick_x;
    }
    rotX = strafe * Math.cos(-yaw / 180 * Math.PI) - drive * Math.sin(-yaw / 180 * Math.PI);
    rotY = strafe * Math.sin(-yaw / 180 * Math.PI) + drive * Math.cos(-yaw / 180 * Math.PI);
    if (gamepad1.circle && cFieldCentric) {
      mImu.ResetYaw();
    }
    // Set max drive power
    // Normal drive speed
    driveMaxPower = cDriveNormal;

    // Scale the output power for the division we are doing later
    sum = Math.abs(drive) + Math.abs(strafe) + Math.abs(rotate);
    if (sum > 1) {
      max = sum;
    } else {
      max = 1;
    }
    // Front left power
    if (cFieldCentric) {
      frontLeftPower = rotY + rotX;
    } else {
      frontLeftPower = drive + strafe;
    }
    frontLeftPower = frontLeftPower + rotate;
    frontLeftPower = frontLeftPower / max;
    // Rear left power
    if (cFieldCentric) {
      rearLeftPower = rotY - rotX;
    } else {
      rearLeftPower = drive - strafe;
    }
    rearLeftPower = rearLeftPower + rotate;
    rearLeftPower = rearLeftPower / max;
    // Front right power
    if (cFieldCentric) {
      frontRightPower = rotY - rotX;
    } else {
      frontRightPower = drive - strafe;
    }
    frontRightPower = frontRightPower - rotate;
    frontRightPower = frontRightPower / max;
    // Rear right power
    if (cFieldCentric) {
      rearRightPower = rotY + rotX;
    } else {
      rearRightPower = drive + strafe;
    }
    rearRightPower = rearRightPower - rotate;
    rearRightPower = rearRightPower / max;
    // Set power to the motors
    MotDrBkLt.setPower(rearLeftPower * driveMaxPower);
    MotDrBkRt.setPower(rearRightPower * driveMaxPower);
    MotDrFrLt.setPower(frontLeftPower * driveMaxPower);
    MotDrFrRt.setPower(frontRightPower * driveMaxPower);
    telemetry.addData("LeftFront", frontLeftPower);
    telemetry.addData("RightFront", frontRightPower);
    telemetry.addData("LeftRear", rearLeftPower);
    telemetry.addData("RightRear", rearRightPower);
  }
}