/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer.CameraDirection;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import android.media.MediaPlayer;
/**
 * This 2019-2020 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Skystone game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Disabled
//@TeleOp(name = "SkystoneDetection", group = "Concept")

public class SkystoneDetection extends LinearOpMode {
    private static final String TFOD_MODEL_ASSET = "Skystone.tflite";
    private static final String LABEL_FIRST_ELEMENT = "Stone";
    private static final String LABEL_SECOND_ELEMENT = "Skystone";
    int SkystonePosition1 = 0;
    //0 is not found
    //1 is left side
    //2 is middle
    //3 is right
    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "AbgqVUj/////AAABmQJK3C3ywkbBs0xOk9ikb0kYt5NS2MeAfNeX/3J/zV8O2s7cr8Y79sdcDs2TBtBquC5T9bmThY0up87lwOqQfuZsq38AfzSRjJZ5qmhPx94e/bkewmh8RgKErgcE0yAbIWS1QwLZKzBFOA2stTpBUiOXBhkX+p07OaFO0sum959QYli9xdmmBZg/9GzusrqedKMcxXY/4+F+H8ui9B49EBrB+MjFZ6Vs796rp4aUKP5xhIqXY1vR6ylvhJHYJtyM/tGmMZB8tgcw2n+p2/S882jafm9PcbYWVYqGvhMswKB225pMZG+R5dotu2kyC7PThRYCOQ+GQ+FJBWTnqT6Nw9w+6FndzDfTRQygi99K2joN";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

    @Override
    public void runOpMode() {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();

        if (ClassFactory.getInstance().canCreateTFObjectDetector()) {
            initTfod();
        } else {
            telemetry.addData("Sorry!", "This device is not compatible with TFOD");
        }

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        float StoneRecognitionx     = 0;
        float SkyStoneRecognitionx  = 0;
        float StonevsSkyStonex      = 0;


        while (opModeIsActive()) {
            if (tfod != null && SkystonePosition1 == 0) {
                boolean stone = false;
                boolean skystone = false;
                // getUpdatedRecognitions() will return null if no new information is available since
                // the last time that call was made.
                List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
                if (updatedRecognitions != null) {
                    telemetry.addData("# Object Detected", updatedRecognitions.size());

                    // step through the list of recognitions and display boundary info.
                    int i = 0;
                    if (updatedRecognitions.size() == 2) {
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            if (recognition.getLabel().equals( "Skystone") && skystone == false) {
                                //Location of the SkyStone
                                skystone = true;
                                SkyStoneRecognitionx = recognition.getTop();
                                telemetry.addData("SkyStoneRecognitionx",SkyStoneRecognitionx);
                            } else if (recognition.getLabel().equals("Stone") && stone == false) {
                                //Location of Stone
                                stone = true;
                                StoneRecognitionx = recognition.getTop();
                                telemetry.addData("StoneRecognitionx", StoneRecognitionx);

                            }

                        }
                        //Subtracting the 2 numbers
                        StonevsSkyStonex = SkyStoneRecognitionx - StoneRecognitionx;
                        telemetry.addData("Difference:", StonevsSkyStonex);
                        if (StoneRecognitionx != 0 && SkyStoneRecognitionx != 0) {
                            if (StonevsSkyStonex > 0) {
                                SkystonePosition1 = 2;
                            } else if (StonevsSkyStonex < 0) {
                                SkystonePosition1 = 1;
                            }
                        } else if (SkyStoneRecognitionx == 0 && StoneRecognitionx != 0) {
                            SkystonePosition1   = 3;
                        }
                        switch (SkystonePosition1) {
                            case 0:
                                telemetry.addLine("Location of SkyStone: Not Known");
                                break;
                            case 1:
                                telemetry.addLine("Location of SkyStone: Left");
                                break;
                            case 2:
                                telemetry.addLine("Location of SkyStone: Middle");
                                break;
                            case 3:
                                telemetry.addLine("Location of SkyStone: Right");
                                break;

                        }
                        telemetry.update();
                    } else {
                        telemetry.addData("Not enough: # Object Detected", updatedRecognitions.size());
                        telemetry.update();
                    }
                }
            }
            if (SkystonePosition1 != 0) {
                telemetry.addLine("Detection made");

                switch (SkystonePosition1) {
                    case 0:
                        telemetry.addLine("Location of SkyStone: Not Known");
                        break;
                    case 1:
                        telemetry.addLine("Location of SkyStone: Left");
                        break;
                    case 2:
                        telemetry.addLine("Location of SkyStone: Middle");
                        break;
                    case 3:
                        telemetry.addLine("Location of SkyStone: Right");
                        break;

                }
                telemetry.update();
            }

        }



        if (tfod != null) {
            tfod.shutdown();
        }


    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraDirection = CameraDirection.BACK;

        //  Instantiate the Vuforia engine
        vuforia = ClassFactory.getInstance().createVuforia(parameters);

        // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
        int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
                "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
        tfodParameters.minimumConfidence = 0.85;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABEL_FIRST_ELEMENT, LABEL_SECOND_ELEMENT);
    }

}
