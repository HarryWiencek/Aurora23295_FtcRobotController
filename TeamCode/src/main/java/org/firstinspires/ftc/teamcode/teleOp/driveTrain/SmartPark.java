package org.firstinspires.ftc.teamcode.teleOp.driveTrain;

import com.acmerobotics.roadrunner.Pose2d;
import com.acmerobotics.roadrunner.SequentialAction;
import com.acmerobotics.roadrunner.TrajectoryActionBuilder;
import com.acmerobotics.roadrunner.Vector2d;
import com.acmerobotics.roadrunner.ftc.Actions;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class SmartPark {

    private final MecanumDrive drive;
    private final org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive dwive;

    public SmartPark(MecanumDrive drive, org.firstinspires.ftc.teamcode.roadrunner.PinpointDrive dwive) {
        this.drive = drive;
        this.dwive = dwive;
    }

    /**
     * Parks the robot to the target position using the most efficient approach.
     * Acceptable final headings are 0, 90, 180, 270 degrees.
     */
    public void parkToTarget(Vector2d target) {
        // Update odometry first
        drive.updateOdo();

        // Get current odometry position
        Pose2D odoPos = drive.getOdoPosition();
        double currentHeading = odoPos.getHeading(org.firstinspires.ftc.robotcore.external.navigation.AngleUnit.DEGREES);
        double robotX = odoPos.getX(DistanceUnit.INCH);
        double robotY = odoPos.getY(DistanceUnit.INCH);

        // Compute angle to target in degrees
        double angleToTarget = Math.toDegrees(Math.atan2(target.y - robotY, target.x - robotX));

        // Snap desired heading to nearest cardinal direction (0, 90, 180, 270)
        double[] cardinalAngles = {0, 90, 180, 270};
        double bestAngle = cardinalAngles[0];
        double minDiff = Math.abs(angleDiff(angleToTarget, cardinalAngles[0]));
        for (double a : cardinalAngles) {
            double diff = Math.abs(angleDiff(angleToTarget, a));
            if (diff < minDiff) {
                minDiff = diff;
                bestAngle = a;
            }
        }

        // Decide whether to drive forward or backward
        double diffToCurrent = angleDiff(bestAngle, currentHeading);
        boolean shouldReverse = Math.abs(diffToCurrent) > 90;

        // Build trajectory
        Pose2d startPoseRR = new Pose2d(robotX, robotY, Math.toRadians(currentHeading));
        TrajectoryActionBuilder parkBuilder = dwive.actionBuilder(startPoseRR);

        if (shouldReverse) {parkBuilder.setReversed(true);}
        parkBuilder.splineTo(target, Math.toRadians(bestAngle));

        // Execute trajectory
        Actions.runBlocking(new SequentialAction(parkBuilder.build()));
    }

    // Helper: returns shortest difference between two angles in degrees [-180, 180]
    private double angleDiff(double target, double current) {
        double diff = target - current;
        diff = ((diff + 180) % 360) - 180;
        return diff;
    }
}
