package org.firstinspires.ftc.teamcode.sample_opmodes;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.cv_libs.FishVisionManager;

/**
 * Created by Justin on 12/7/2017.
 * All code handled by Justin.
 */

public class SampleAuto extends FishVisionManager {
    private JewelSide jewelSide;
    private final double TIME_OUT = 2500; //Time you try and scan for the jewel around 2 seconds is good
    @Override
    public void runOpMode() throws InterruptedException {
        StartFishVisionBackEnd(); //This will turn the flashlight on and add telemetry data.

        jewelSide = findJewel(TIME_OUT);

        telemetry.addData("Found jewel is",jewelSide.name());
        telemetry.update();

        setFlashOff(); //Turn flash off to save battery.

        while (opModeIsActive()); //Prevent the program from closing to fast.
    }
}
