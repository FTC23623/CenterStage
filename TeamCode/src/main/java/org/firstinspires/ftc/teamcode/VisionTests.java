package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.datalogger.HydraObjDetDatalogger;
import org.firstinspires.ftc.teamcode.objects.HydraOpMode;
import org.firstinspires.ftc.teamcode.subsystems.HydraObjectDetect;
import org.firstinspires.ftc.teamcode.types.HydraObjectLocations;
import java.util.List;

@TeleOp(name = "VisionTests")
public class VisionTests extends LinearOpMode {
    HydraObjectDetect mObjDet;
    HydraOpMode mOpmode;
    HydraObjDetDatalogger mObjDetDatalogger;
    @Override
    public void runOpMode() throws InterruptedException {
        mObjDet = new HydraObjectDetect(mOpmode, "Blue_Prop.tflite");
        mObjDetDatalogger = new HydraObjDetDatalogger("obj-log-");
        mOpmode = new HydraOpMode(telemetry, hardwareMap, null, mObjDetDatalogger);
        List<LynxModule> hubs = hardwareMap.getAll(LynxModule.class);
        for (LynxModule mod : hubs) {
            mod.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
        while (!mObjDet.CameraIsReady()) {
            if (isStopRequested()) {
                break;
            }
        }
        waitForStart();
        while (opModeIsActive()) {
            mObjDet.GetObjectLocation(false);
            mObjDet.FindAprilTag(HydraObjectLocations.ObjLocBlueRightSpike);
            telemetry.update();
            sleep(20);
        }
    }
}
