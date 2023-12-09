package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedBackstageFront", preselectTeleOp = "HyDrive")
public class HydrAuton_RedBackstageFront extends HydrAuton_BackstageFront {
    public HydrAuton_RedBackstageFront() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = false;
        modelFilename = "Red_Prop.tflite";
        mOpModeName = "Red-Backstage-Front";
    }
}
