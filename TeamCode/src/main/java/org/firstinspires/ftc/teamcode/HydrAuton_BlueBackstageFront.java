package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Autonomous(name = "BlueBackstageFront", preselectTeleOp = "HyDrive")
@Disabled
public class HydrAuton_BlueBackstageFront extends HydrAuton_BackstageFront {
    public HydrAuton_BlueBackstageFront() {
        setTrueForRed = false;
        setTrueForRiggingOnRight = true;
        modelFilename = "Blue_Prop.tflite";
        mOpModeName = "Blue-Backstage-Front";

    }
}
