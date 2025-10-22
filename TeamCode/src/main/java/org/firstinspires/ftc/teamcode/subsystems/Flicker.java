package org.firstinspires.ftc.teamcode.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.acmerobotics.roadrunner.ftc.Actions;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.ElapsedTime;

public class Flicker {
    // superspeed servo
    public final ServoImplEx flicker;

    public Flicker(HardwareMap hwMap) {
        flicker = (ServoImplEx) hwMap.get(Servo.class, "flicker");
        flicker.setPwmRange(new PwmControl.PwmRange(600, 2400));
    }

    public class Flick implements Action {
        private boolean initialized = false;
        private final ElapsedTime timer = new ElapsedTime();
        private double expectedTime;
        private final double target;

        public Flick(double target) {
            this.target = target;
        }

        @Override
        public boolean run(@NonNull TelemetryPacket packet) {
            if (!initialized) {
                timer.reset();
                expectedTime = 1;
                flicker.setPosition(0);
                initialized = true;
            }

            flicker.setPosition(target);
            flicker.setPosition(0);
            return timer.seconds() <= expectedTime;
        }

    }

    public Action flick(double target) {
        return new Flick(target);
    }
}