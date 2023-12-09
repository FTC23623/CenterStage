package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "RedWing", preselectTeleOp = "HyDrive")
public class HydrAuton_RedWing extends HydrAuton_Wing {
    public HydrAuton_RedWing() {
        setTrueForRed = true;
        setTrueForRiggingOnRight = true;
        modelFilename = "Red_Prop.tflite";
        mOpModeName = "Red-Wing";
    }
}
