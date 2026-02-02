package org.firstinspires.ftc.teamcode;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Configurable
@Autonomous(name = "Blue Autonomous", group = "Robot")
public class BlueAuto {
private Follower follower;
private Timer timer;
private Pose currentPose;
private int pathState = 0; // finite state machine variable
private boolean init = true;

}
