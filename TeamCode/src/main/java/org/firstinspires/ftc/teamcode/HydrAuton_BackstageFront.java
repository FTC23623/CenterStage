package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.teamcode.types.HydraArmMovements;

public class HydrAuton_BackstageFront extends HydrAuton {
    public boolean RunAuton() {
        if (autonState < 100) {
            // These states handle driving to the correct spike
            if (!AutonDriveToSpike(setTrueForRiggingOnRight)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 200) {
            // These 100 level states handle dropping the pixel on the spike
            // this is the same for all autons
            if (!PixelDrop(false)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 300) {
            // These 200 level states handle driving to the backdrop
            if (!AutonDriveToBackdropFromBackstageFront(setTrueForRed)) {
                BadState();
                return true;
            }
        }
        else if (autonState < 400) {
            // These 300 level states handle scoring backwards at the backdrop
            // this is the same for two autons
            if (!ScoreFront()) {
                BadState();
                autonState = 400;
            }
        }
        else if (autonState < 500) {
            // These 400 level states handle returning the arm home
            // this is the same for all autons
            ArmToHome();
        }
        else if (autonState < 600) {
            switch (autonState) {
                case 500:
                    if (!Drive.Busy()) {
                        int dist;
                        switch (ObjLoc) {
                            case ObjLocRedLeftSpike:
                                dist = 34;
                                break;
                            case ObjLocBlueLeftSpike:
                                dist = -22;
                                break;
                            case ObjLocBlueCenterSpike:
                                dist = -30;
                                break;
                            case ObjLocRedCenterSpike:
                                dist = 30;
                                break;
                            case ObjLocRedRightSpike:
                            default:
                                dist = 22;
                                break;
                            case ObjLocBlueRightSpike:
                                dist = -34;
                                break;
                        }
                        Drive.Start(0, dist, 0, mHeading);
                        autonState += 1;
                    }
                    break;
                case 501:
                    if (!Drive.Busy()) {
                        Drive.Start(14, 0, 0, mHeading);
                        autonState += 1;
                    }
                    break;
                case 502:
                    if (!Drive.Busy()) {
                        autonState = 600;
                    }
                    break;
                default:
                    BadState();
                    return false;
            }
        }
        else if (autonState == 600) {
            // this auton is complete
            return true;
        }
        else {
            BadState();
            return true;
        }
        return false;
    }

    /**
     * Drive the robot to the backdrop from the backstage based on which spike we delivered at [ObjLoc]
     * This function is used for both backstage side auton op modes
     * @param flipForRed is set to true if we are running a red team auton
     * @return false iff there is a problem
     */
    protected boolean AutonDriveToBackdropFromBackstageFront(boolean flipForRed) {
        // multiply strafes and rotates by -1 based on the starting orientation
        int flip = 1;
        if (flipForRed) {
            flip = -1;
        }
        switch (autonState) {
            case 200:
                // jump to the correct state based on which spike we are at
                switch (ObjLoc) {
                    case ObjLocBlueLeftSpike:
                    case ObjLocRedRightSpike:
                        autonState = 210;
                        break;
                    case ObjLocBlueCenterSpike:
                    case ObjLocRedCenterSpike:
                        autonState = 220;
                        break;
                    case ObjLocBlueRightSpike:
                    case ObjLocRedLeftSpike:
                        autonState = 230;
                        break;
                    default:
                        return false;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 210:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-4, 0, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 211:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    mHeading = 90 * flip;
                    Drive.Start(0, 0, -20 * flip, mHeading);
                    autonState += 1;
                }
                break;
            case 212:
                // BLUE LEFT SPIKE
                // RED RIGHT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(18, 8 * flip, 0, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 220:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-14, 10 * flip, 0, mHeading);
                    autonState += 1;
                }
                break;
            case 221:
                // CENTER SPIKE
                if (!Drive.Busy()) {
                    mHeading = 90 * flip;
                    Drive.Start(0, 0, -40, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ////////////////////////////////////////////////////////////////////////////////////////
            case 230:
                // BLUE RIGHT SPIKE
                // RED LEFT SPIKE
                if (!Drive.Busy()) {
                    Drive.Start(-27, -2 * flip, 0, mHeading);
                    autonState = 231;
                }
                break;
            case 231:
                // BLUE RIGHT SPIKE
                // RED LEFT SPIKE
                if (!Drive.Busy()) {
                    mHeading = 90 * flip;
                    Drive.Start(0, 0, -40, mHeading);
                    Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                    autonState = 299;
                }
                break;
            ///////////////////////////////////////////////////////////////////////////////////////
            case 299:
                boolean drivecomplete = !Drive.Busy();
                boolean armcomplete = Arm.RunAction(HydraArmMovements.ArmMoveToFront);
                if (drivecomplete && armcomplete) {
                    autonState = 300;
                }
                break;
            default:
                // something bad happened
                return false;
        }
        if (autonState >= 200) {
            // now that we are driving towards the backdrop, start looking for the AprilTag
            // this just prints to telemetry for now
            ObjDet.FindAprilTag(ObjLoc);
        }
        return true;
    }
}
