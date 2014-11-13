package com.example.spiri_android.spiri_myo_commander;

import com.thalmic.myo.Pose;

import org.ros.concurrent.CancellableLoop;
import org.ros.namespace.GraphName;
import org.ros.node.ConnectedNode;
import org.ros.node.Node;
import org.ros.node.NodeMain;
import org.ros.node.topic.Publisher;


/**
 * A node which turns Myo Poses into Twists and publishes them on /cmd_vel
 *
 * Author: Arnold
 */
public class MyoNode implements NodeMain {

    private geometry_msgs.Twist current_velocity_command;
    private std_msgs.String current_gesture;
    private Pose current_pose = Pose.UNKNOWN;

    private boolean enable = false;

    public void setMyoDisconnected(){
        current_velocity_command.getLinear().setX(0.0);
        current_velocity_command.getLinear().setY(0.0);
        current_velocity_command.getLinear().setZ(0.0);
        current_velocity_command.getAngular().setX(0.0);
        current_velocity_command.getAngular().setY(0.0);
        current_velocity_command.getAngular().setZ(0.0);
        current_gesture.setData("disconnected");
    }


    public void updateOrientation(double roll, double pitch, double yaw){
        if (enable){
            current_velocity_command.getLinear().setX( (pitch - 15 ) / 60);
            current_velocity_command.getLinear().setY( (roll - 60)/ 60 );
            current_velocity_command.getLinear().setZ(0.0);
            current_velocity_command.getAngular().setX(0.0);
            current_velocity_command.getAngular().setY(0.0);
            current_velocity_command.getAngular().setZ(Math.PI/180*yaw);
        }
    }

    public void updateGesture(Pose pose){
        current_gesture.setData(pose.name());
        current_pose = pose;

        switch (pose) {
            case UNKNOWN:
                enable = false;
                current_velocity_command.getLinear().setX(0.0);
                current_velocity_command.getLinear().setY(0.0);
                current_velocity_command.getLinear().setZ(0.0);
                current_velocity_command.getAngular().setX(0.0);
                current_velocity_command.getAngular().setY(0.0);
                current_velocity_command.getAngular().setZ(0.0);
                break;
            case REST:
                enable = false;
                current_velocity_command.getLinear().setX(0.0);
                current_velocity_command.getLinear().setY(0.0);
                current_velocity_command.getLinear().setZ(0.0);
                current_velocity_command.getAngular().setX(0.0);
                current_velocity_command.getAngular().setY(0.0);
                current_velocity_command.getAngular().setZ(0.0);
                break;
            case FIST:
                // Allow IMU based commands
                enable = true;
                break;
            case THUMB_TO_PINKY:
                enable = false;
                // Go up
                current_velocity_command.getLinear().setX(0.0);
                current_velocity_command.getLinear().setY(0.0);
                current_velocity_command.getLinear().setZ(1.0);
                current_velocity_command.getAngular().setX(0.0);
                current_velocity_command.getAngular().setY(0.0);
                current_velocity_command.getAngular().setZ(0.0);
                break;
            case FINGERS_SPREAD:
                enable = false;
                // Go down
                current_velocity_command.getLinear().setX(0.0);
                current_velocity_command.getLinear().setY(0.0);
                current_velocity_command.getLinear().setZ(-1.0);
                current_velocity_command.getAngular().setX(0.0);
                current_velocity_command.getAngular().setY(0.0);
                current_velocity_command.getAngular().setZ(0.0);
                break;
        }
    }

    public geometry_msgs.Twist getCurrentVelocityCommand(){
        return current_velocity_command;
    }

    public Pose getCurrentPose() {return current_pose; }

    private geometry_msgs.Twist applyDeadband(geometry_msgs.Twist command_in) {
        // TODO(Arnold): Parameterize the dead-band parameters
        geometry_msgs.Twist command_out = command_in;

        command_out.getAngular().setX(0);
        command_out.getAngular().setY(0);

        if (Math.abs(command_out.getAngular().getZ()) < 0.3)
            command_out.getAngular().setZ((0.0));

        if (Math.abs(command_out.getLinear().getX()) < 0.1)
            command_out.getLinear().setX(0.0);
        if (Math.abs(command_out.getLinear().getY()) < 0.1)
            command_out.getLinear().setY(0.0);
        if (Math.abs(command_out.getLinear().getZ()) < 0.1)
            command_out.getLinear().setZ(0.0);

        return command_out;
    }

    @Override
    public GraphName getDefaultNodeName() {
        return GraphName.of("spiri_myo_commander");
    }

    @Override
    public void onStart(final ConnectedNode connectedNode) {
        final Publisher<geometry_msgs.Twist> cmd_vel_publisher =
                connectedNode.newPublisher("cmd_vel", geometry_msgs.Twist._TYPE);
        final Publisher<std_msgs.String> gesture_publisher =
                connectedNode.newPublisher("gesture", std_msgs.String._TYPE);
        // This CancellableLoop will be canceled automatically when the node shuts
        // down.
        connectedNode.executeCancellableLoop(new CancellableLoop() {

            @Override
            protected void setup() {
                //TODO(Arnold): Make sure these get properly initialized to 0
                current_velocity_command = cmd_vel_publisher.newMessage();
                current_gesture = gesture_publisher.newMessage();
            }

            @Override
            protected void loop() throws InterruptedException {
                cmd_vel_publisher.publish(applyDeadband(current_velocity_command));
                gesture_publisher.publish(current_gesture);
                Thread.sleep(300);
            }
        });
    }

    @Override
    public void onShutdown(Node node) {

    }

    @Override
    public void onShutdownComplete(Node node) {

    }

    @Override
    public void onError(Node node, Throwable throwable) {

    }
}