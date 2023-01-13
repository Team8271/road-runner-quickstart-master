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

package org.firstinspires.ftc.teamcode.drive;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;

import java.util.List;

/*
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousBooleanOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousIntOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousOption;
import org.firstinspires.ftc.teamcode.AutoOptions.AutonomousTextOption;
*/


/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "BasicParkTony", group = "TFOD-Autos")
//@Disabled
public class BasicParkTony extends LinearOpMode {
    
    TonyRobotHardware robot = new TonyRobotHardware();
    
    //region Initialize TFOD and VuForia
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: ,
     *  1: Cube,
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "PPDEC_8.tflite";
    private static final String[] LABELS = {
            "bd",
            "gh",
            "rw"
    };

    //this string is going to return a value based on the signal cone later in the code
    String signal = null;

    /*
    int bottomLevel = 350;
    int approachBottom = -390;

    int middleLevel = 600;
    int approachMiddle = -400;

    int topLevel = 1100;
    int approachTop = -600;
*/

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
            "Ad/WJUD/////AAABmaHL2VlsskF7gs94CBhc85VMCMsnduT4r56lJ6R1ADz06l0nCXlkYHuEr/9MViHanSiKcefbD5RMEKuNSbMTOmC8JGbEkiQB5a+kE/JDCayLu/0cAj7+y4wkNo2v4YtJlr1YJ5HCLZ1Rzv007cx4S+NbSv3TSxZUQzomnBbZIc/3uLx5S0Sr3eood8gq7xRVTwXh0Rp9GJk+my8sz87vJyg+nZlWXa3q5WzuS0YRq2F5XMDMH1opYjN3Ub+0xFIZO82tBSBQfAMGLruFRyjQ7qpVgPra19wu8PldMmHoGHPdQgT+G6iAGCjClGpcnPtZMXw1VycsGRyjH4pBSH12J5HIheL9b/BTvvBwelC+0FeC";

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

    private void initVuforia() {
        /*
         * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
         */
        VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

        parameters.vuforiaLicenseKey = VUFORIA_KEY;
        parameters.cameraName = hardwareMap.get(WebcamName.class, "Webcam 1");

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
        tfodParameters.minResultConfidence = 0.8f;
        tfodParameters.isModelTensorFlow2 = true;
        tfodParameters.inputSize = 320;
        tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
        tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
    //endregion

    //region analyze signal
    public String analyzeSignal()
    {
        if(tfod != null)
        {
            List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

            if(updatedRecognitions != null)
            {
                for (Recognition recognition : updatedRecognitions)
                {
                    if(recognition.getConfidence() > .89)
                    {
                        if(recognition.getLabel().equals("bd"))
                        {
                            //return blue circles
                            return "A";

                        }
                        else if(recognition.getLabel().equals("gh"))
                        {
                                //return green hook shapes
                                return "B";
                        }
                    }
                }
            }
        }
        //if the signal cone displays neither green hooks nor the blue circles, assume it's the red squiggly lines!
        return "C";
    }
    //endregion

    /*


    For each auto option the parameters are essentially 1- the label to show on the driver station, 2 - starting value, 3 - the possible values
    AutonomousTextOption    allianceColor       = new AutonomousTextOption("Alliance Color", "blue", new String[] {"blue", "red"});
    AutonomousTextOption    startPos       = new AutonomousTextOption("Start Position", "X", new String[] {"X", "Y"});
    AutonomousTextOption    endPos = new AutonomousTextOption("End Position", "storage", new String[] {"storage","warehouse"});
    AutonomousIntOption     waitStart           = new AutonomousIntOption("Wait at Start", 0, 0, 20);

    //This is the order of our options and setting them all to their preset value.
    AutonomousOption[] autoOptions       = {allianceColor, startPos, endPos, waitStart};
    int currentOption = 0;

    //this setting the buttons to false to make sure options are not being chosen for us.
    boolean aPressed = false;
    boolean bPressed = false;
    boolean xPressed = false;
    boolean yPressed = false;

    //region Autonomous Options
    // This is how we get our autonomous options to show up on our phones.
    public void showOptions (){
        int index = 0;
        String str = "";

        while (index < autoOptions.length){
            switch (autoOptions[index].optionType){
                case STRING:
                    str = ((AutonomousTextOption)autoOptions[index]).getValue();
                    break;
                case INT:
                    str = Integer.toString(((AutonomousIntOption)autoOptions[index]).getValue());
                    break;
                case BOOLEAN:
                    str = String.valueOf(((AutonomousBooleanOption)autoOptions[index]).getValue());
                    break;
            }

            if (index == currentOption){
                telemetry.addData(Integer.toString(index) + ") ==> " + autoOptions[index].name,str);
            } else {
                telemetry.addData(Integer.toString(index) + ")     " + autoOptions[index].name, str);
            }

            index = index + 1;
        }
        telemetry.update();
    }
    // This is how we select our auto options
    public void selectOptions () {

        while (currentOption< autoOptions.length && !opModeIsActive()){
            showOptions();

            if (gamepad1.a && !aPressed) {
                currentOption = currentOption + 1;
            }
            aPressed = gamepad1.a;

            if (gamepad1.y && !yPressed) {
                currentOption = currentOption - 1;
            }
            yPressed = gamepad1.y;

            if (gamepad1.b && !bPressed) {
                autoOptions[currentOption].nextValue();
            }
            bPressed = gamepad1.b;

            if (gamepad1.x && !xPressed) {
                autoOptions[currentOption].previousValue();
            }
            xPressed = gamepad1.x;

            telemetry.update();
            Thread.yield();
        }

        telemetry.addData("Robot","READY!!");
        telemetry.update();
    }
    */


    //Depending on the return of the signal, we will call these three methods (see lines 325-345.)

    private void Zone1() throws InterruptedException
    {
        //our robot will always start with the same opening moves, AKA, AutoOpening
        AutoOpening();
        //then, depending on the picture on the signal cone, this code will continue to the correct parking space, parking zone 1
    }

    private void Zone2() throws InterruptedException
    {
        AutoOpening();
        //park in zone 2
    }

    private void Zone3() throws InterruptedException
    {
        AutoOpening();
        //park in zone 3
    }

    private void AutoOpening() throws InterruptedException
    {
        // Below are the opening moves of our autonomous program.

        swingArm(0.3, 300);
        DriveForwardEncoder(0.4, 850);
        DriveLift(1, 1750);
            StopDrivingTime(1);
        SpinLeftIMU(0.3, 45);
            StopDrivingTime(1);
        DriveForwardEncoder(0.2, 275);
            StopDrivingTime(1);
        openPalms();
            StopDrivingTime(1/2);
        DriveBackwardEncoder(0.3, 275);
            StopDrivingTime(1);
        SpinRightIMU(0.3, 0);
            StopDrivingTime(1);
        DriveForwardEncoder(0.3, 900);
            StopDrivingTime(1);
        SpinRightIMU(0.3, 90);
            StopDrivingTime(1/2);
        DriveForwardEncoder(DRIVE_POWER, 1000);
        DriveLift(1, 0);
        swingArm(-0.3, -300);
            StopDrivingTime(1);
        closePalms();
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();
        robot.init(hardwareMap);

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
            tfod.activate();

            tfod.setZoom(1.0, 16.0/9.0);

        }

        //selectOptions();

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();
        //sleep(waitStart.getValue()*1000);

        signal = analyzeSignal();
        {
            //If returns squiggly lines
            if (signal.equals("C")) {
                Zone1();
            }

            //If returns green hooks
            else if (signal.equals("B")) {
                Zone2();
            }

            //If returns blue circles
            else if (signal.equals("A")){
                Zone3();
            }
        }

        //region main while-loop and TFOD telemetry

        if (opModeIsActive()) {
            while (opModeIsActive()) {

                liftHold();

                getRawHeading();

                telemetry.addData("Hold Pos: ", liftRHoldPos);

                if (tfod != null) {
                    signal = analyzeSignal();

                    //I want to add this to display what barcode position it's on, however I'm unsure where to put it
                    telemetry.addData("Position", signal);
                    telemetry.update();

                    // getUpdatedRecognitions() will return null if no new information is available since
                    // the last time that call was made.


                    List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();

                    if (updatedRecognitions != null) {
                        telemetry.addData("# Object Detected", updatedRecognitions.size());

                        // step through the list of recognitions and display boundary info.
                        int i = 0;
                        for (Recognition recognition : updatedRecognitions) {
                            telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                           /*
                            telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f",
                                    recognition.getLeft(), recognition.getTop());
                            telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f",
                                    recognition.getRight(), recognition.getBottom());
                            */
                            i++;

                        }
                    }
                }
            }
        }
        //endregion
    }

    /**
     *      METHODS FOR         !!!!!!
     *      AUTONOMOUS DRIVE,   !!!!!!
     *      ARM, & IMU          !!!!!!
     **/

    //Default Drive Power
    double DRIVE_POWER = 0.5;

    //Wait timer for StopDrivingTime function
    public ElapsedTime waitTime = new ElapsedTime();

    /**   IMU    **/
    //region IMU
    // read the raw (un-offset Gyro heading) directly from the IMU
    public double getRawHeading() {
        Orientation angles   = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);
        return angles.firstAngle;
    }

    //Reset the "offset" heading back to zero
    public void resetHeading() {
        // Save a new heading offset equal to the current raw heading.
        robot.headingOffset = getRawHeading();
        robot.robotHeading = 0;
    }

    public double getSteeringCorrection(double desiredHeading, double proportionalGain) {
        robot.targetHeading = desiredHeading;  // Save for telemetry

        // Get the robot heading by applying an offset to the IMU heading
        robot.robotHeading = getRawHeading() - robot.headingOffset;

        // Determine the heading current error
        robot.headingError = robot.targetHeading - robot.robotHeading;

        // Normalize the error to be within +/- 180 degrees
        while (robot.headingError > 180)  robot.headingError -= 360;
        while (robot.headingError <= -180) robot.headingError += 360;

        // Multiply the error by the gain to determine the required steering correction/  Limit the result to +/- 1.0
        return Range.clip(robot.headingError * proportionalGain, -1, 1);
    }

    public void SpinRightIMU(double power, float angle) throws InterruptedException
    {
        resetHeading();

        //add 10 to -angle to fix overcorrection
        robot.targetHeading = -angle + 10;

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);

        while(getRawHeading() > robot.targetHeading)
        {
            liftHold();

            getRawHeading();
            /*            getSteeringCorrection(robot.P_TURN_GAIN, robot.heading);*/

            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", robot.targetHeading, getRawHeading());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void SpinLeftIMU(double power, float angle) throws InterruptedException
    {
        resetHeading();

        //subtract 10 from angle to fix overcorrection
        robot.targetHeading = angle - 10;

        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);

        while(getRawHeading() < robot.targetHeading)
        {
            liftHold();

            getRawHeading();
            /*            getSteeringCorrection(robot.P_TURN_GAIN, robot.heading);*/

            telemetry.addData("Angle Target:Current", "%5.2f:%5.0f", robot.targetHeading, getRawHeading());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }
    //endregion


    /**      ENCODER       **/
    //region encoder drive
    public void StopDriving()
    {
        robot.frontRight.setPower(0);
        robot.backRight.setPower(0);
        robot.frontLeft.setPower(0);
        robot.backLeft.setPower(0);
    }

    public void StopDrivingTime(long time)  throws InterruptedException
    {
        waitTime.reset();
        waitTime.startTime();

        while(waitTime.time() < time)
        {
            liftHold();

            robot.frontRight.setPower(0);
            robot.frontLeft.setPower(0);
            robot.backRight.setPower(0);
            robot.backLeft.setPower(0);
        }
    }

    private void readyEncoders(int pos) {

        //The code below

        robot.frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //robot.frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.frontRight.setTargetPosition(pos);
        robot.backRight.setTargetPosition(pos);
        //robot.frontLeft.setTargetPosition(pos);
        robot.backLeft.setTargetPosition(pos);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        //robot.frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        robot.frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        //robot.frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        robot.backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void StrafeRightEncoder(double power, int pos) throws InterruptedException
    {
        readyEncoders(pos);

        robot.frontRight.setPower(power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(-power);

        while((robot.backLeft.getCurrentPosition() > pos) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("FRmotorPos", robot.frontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.backRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void StrafeLeftEncoder(double power, int pos) throws InterruptedException
    {
        readyEncoders(pos);

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(power);

        while((robot.backLeft.getCurrentPosition() < pos) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("FRmotorPos", robot.frontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.backRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void DriveBackwardEncoder(double power, int pos) throws InterruptedException
    {
        readyEncoders(pos);

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);

        while((Math.abs(robot.backLeft.getCurrentPosition()) < pos) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("FRmotorPos", robot.frontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.backRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void DriveForwardEncoder(double power, int pos) throws InterruptedException
    {
        readyEncoders(pos);

        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);

        while((Math.abs(robot.backLeft.getCurrentPosition()) < pos) && opModeIsActive())
        {
            liftHold();

            telemetry.addData("FRmotorPos", robot.frontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.backRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void SpinRightEncoder(double power, int pos) throws InterruptedException
    {
        //750 is a 90 degree left turn

        readyEncoders(pos);

        robot.frontRight.setPower(-power);
        robot.backRight.setPower(-power);
        robot.frontLeft.setPower(power);
        robot.backLeft.setPower(power);

        while(robot.backLeft.getCurrentPosition() > pos)
        {
            liftHold();

            telemetry.addData("FRmotorPos", robot.frontRight.getCurrentPosition());
            telemetry.addData("FLmotorPos", robot.frontLeft.getCurrentPosition());
            telemetry.addData("BRmotorPos", robot.backRight.getCurrentPosition());
            telemetry.addData("BLmotorPos", robot.backLeft.getCurrentPosition());
            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();

    }

    public void SpinLeftEncoder(double power, int pos) throws InterruptedException
    {
        //-750 is a 90 degree right turn

        readyEncoders(pos);

        robot.frontRight.setPower(power);
        robot.backRight.setPower(power);
        robot.frontLeft.setPower(-power);
        robot.backLeft.setPower(-power);

        while(robot.angles.firstAngle < 90)
        {
            liftHold();

            telemetry.addData("Heading", robot.angles.firstAngle);

            telemetry.update();
        }

        //turn motor power to 0
        StopDriving();
    }

    //endregion


    /**      ARM & HAND       **/
    //region Arm & Hand Functions
    public void openPalms() throws InterruptedException
    {
        robot.palmL.setPosition(0.5);
        robot.palmR.setPosition(0.5);
    }

    public void closePalms() throws InterruptedException
    {
        robot.palmL.setPosition(1);
        robot.palmR.setPosition(0);
    }

    public void swingArm(double power, int pos) throws InterruptedException
    {
        //sets arm's starting pos to 0
        robot.armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        robot.armMotor.setTargetPosition(pos);
        robot.armMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        robot.armMotor.setPower(power);
        while(robot.armMotor.isBusy())
        {
            telemetry.addData("armPos", robot.armMotor.getCurrentPosition());
            telemetry.update();
        }
    }

    int liftRHoldPos;

    public void liftHold() throws InterruptedException
    {
        //set BOTH of them to the lift L hold pos to avoid accidentally running them both at different values
        robot.liftL.setPower((double)(liftRHoldPos - robot.liftR.getCurrentPosition()) / robot.slopeVal);
        robot.liftR.setPower((double)(liftRHoldPos - robot.liftR.getCurrentPosition()) / robot.slopeVal);
    }

    public void DriveLift(double power, int pos)
    {
        robot.liftL.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        robot.liftR.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        while((robot.liftL.getCurrentPosition() < pos) && opModeIsActive())
        {
            robot.liftL.setPower(power);
            robot.liftR.setPower(power);
        }

        //update arm hold position
        liftRHoldPos = pos;

        robot.liftL.setPower(0);
        robot.liftR.setPower(0);
    }
    //endregion
}
