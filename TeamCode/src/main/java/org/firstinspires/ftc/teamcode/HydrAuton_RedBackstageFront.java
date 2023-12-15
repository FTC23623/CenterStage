package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "RedBackstageFront", preselectTeleOp = "HyDrive")
@Disabled
public class HydrAuton_RedBackstageFront extends HydrAuton_BackstageFront {
    public HydrAuton_RedBackstageFront() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = false;
        modelFilename = "Red_Prop.tflite";
        mOpModeName = "Red-Backstage-Front";
    }
}
