package org.firstinspires.ftc.teamcode.autonomous;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.ParallelAction;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.PinpointDrive;

import org.firstinspires.ftc.teamcode.subsystems.Launcher;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Flicker;


@Config
@Autonomous(name = "AUTONOMOUS_BLUE", group = "Auto")
public class Blue extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Pose2d initialPose = new Pose2d(-9, -48, Math.toRadians(90));
        PinpointDrive drive = new PinpointDrive(hardwareMap, initialPose);
        Launcher launcher = new Launcher(hardwareMap);
        Intake intake = new Intake(hardwareMap);
        Flicker flicker = new Flicker(hardwareMap);

        TrajectoryActionBuilder traj1, traj2;

        //traj1 = drive.actionBuilder(initialPose).strafeTo(new Vector2d(0, 0));
        //traj2 = traj1.endTrajectory().fresh().strafeToLinearHeading(new Vector2d(32, -39), Math.toRadians(90));
        traj2 = drive.actionBuilder(initialPose).strafeToLinearHeading(new Vector2d(-32, -39), Math.toRadians(90));
        Actions.runBlocking(
                new SequentialAction(
                        //traj1.build(),
                        new ParallelAction(
                            launcher.launch(1, 3),
                            flicker.flick(1)
                        ),
                        traj2.build()
                )
        );

        telemetry.addData("Path", "Execution complete");
        telemetry.update();
    }
}