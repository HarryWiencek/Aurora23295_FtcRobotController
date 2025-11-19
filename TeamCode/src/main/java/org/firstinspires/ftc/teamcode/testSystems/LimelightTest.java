package org.firstinspires.ftc.teamcode.testSystems;

import static android.os.SystemClock.sleep;

import android.content.Context;
import android.content.SharedPreferences;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;
import org.firstinspires.ftc.teamcode.teleOp.driveTrain.MecanumDrive;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Limelight;
import org.firstinspires.ftc.teamcode.teleOp.subSystems.Mosaic;

@TeleOp(name = "LimelightTest", group = "TestModes")
public class LimelightTest extends OpMode {

    private Limelight limelight;
    private Mosaic detectedPattern;
    private MecanumDrive drive = new MecanumDrive();

    @Override
    public void init() {
        limelight = new Limelight(hardwareMap, 0);
        drive.init(hardwareMap, telemetry);

        telemetry.addLine("Initialization complete");
        telemetry.update();

        SharedPreferences prefs = hardwareMap.appContext
                .getSharedPreferences("RobotPrefs", Context.MODE_PRIVATE);
        String name = prefs.getString("detectedMosaic", "UNKNOWN");
        detectedPattern = Mosaic.valueOf(name);

    }

    @Override
    public void init_loop() {
        // Scan for obelisk pattern
        if (detectedPattern == Mosaic.UNKNOWN)
            detectedPattern = limelight.scanObelisk();

        switch (detectedPattern) {
            case UNKNOWN:
                telemetry.addLine("No tag detected");
                break;
            default:
                telemetry.addData("Obelisk Pattern", detectedPattern.name());
                break;
        }
        telemetry.update();

        sleep(50);

    }

    @Override
    public void loop() {
        if (detectedPattern == Mosaic.UNKNOWN) {
            detectedPattern = limelight.scanObelisk();
            sleep(30);
        }

        // Switch Limelight pipeline based on gamepad input
        if (gamepad1.triangleWasPressed()) limelight.changePipeline(1);
        if (gamepad1.squareWasPressed()) limelight.changePipeline(0);

        // Get robot heading from odometry
        double heading = drive.getOdoHeading(AngleUnit.DEGREES);

        // Get robot poses
        Pose2D botPoseMT1 = limelight.get2DLocationMT1();
        Pose2D botPoseMT2 = limelight.get2DLocation(heading);
        Pose3D botPoseMT2_3D = limelight.getLocation(heading);
        Pose2D drivePose = drive.getOdoPosition();

        // Add nicely formatted telemetry
        telemetry.addData("MT1 Pose2D", formatPose2D(botPoseMT1));
        telemetry.addData("MT2 Pose2D", formatPose2D(botPoseMT2));
        telemetry.addData("MT2 Pose3D", formatPose3D(botPoseMT2_3D));
        telemetry.addData("Drive Pose", formatPose2D(drivePose));

        telemetry.addLine();

        if (detectedPattern == Mosaic.UNKNOWN) {
            telemetry.addLine("No tag detected");
        } else {
            telemetry.addData("Obelisk Pattern", detectedPattern.name());
        }

        telemetry.update();
    }

    @Override
    public void stop() {
        super.stop();
    }

    // Helper to format Pose2D to 2 decimal places
    private String formatPose2D(Pose2D pose) {
        if (pose != null) {
            String data = String.format("X: %.2f, Y: %.2f, Heading: %.2f deg",
                    pose.getX(DistanceUnit.INCH), pose.getY(DistanceUnit.INCH),
                    pose.getHeading(AngleUnit.DEGREES));
            return data;
        } else {
            return "Null";
        }
    }

    // Helper to format Pose3D to 2 decimal places
    private String formatPose3D(Pose3D pose) {
        if (pose != null) {
            String data = String.format("X: %.2f, Y: %.2f, Z: %.2f, Roll: %.2f, Pitch: %.2f, Heading: %.2f deg",
                    pose.getPosition().x, pose.getPosition().y, pose.getPosition().z,
                    pose.getOrientation().getRoll(), pose.getOrientation().getPitch(), pose.getOrientation().getYaw());
            return data;
        } else {
            return "Null";
        }

    }


}
