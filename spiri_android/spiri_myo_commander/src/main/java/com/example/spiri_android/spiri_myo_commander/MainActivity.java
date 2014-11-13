package com.example.spiri_android.spiri_myo_commander;

/*
 * Copyright (C) 2011 Google Inc.
 *
 * Licensed under the Apache License, Version 2.0 (the "License"); you may not
 * use this file except in compliance with the License. You may obtain a copy of
 * the License at
 *
 * http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
 * WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
 * License for the specific language governing permissions and limitations under
 * the License.
 */

import android.app.Activity;
import android.bluetooth.BluetoothAdapter;
import android.content.Intent;
import android.graphics.Color;
import android.os.Bundle;
import android.view.Menu;
import android.view.MenuInflater;
import android.view.MenuItem;
import android.widget.TextView;
import android.widget.Toast;


import com.thalmic.myo.AbstractDeviceListener;
import com.thalmic.myo.Arm;
import com.thalmic.myo.DeviceListener;
import com.thalmic.myo.Hub;
import com.thalmic.myo.Myo;
import com.thalmic.myo.Pose;
import com.thalmic.myo.Quaternion;
import com.thalmic.myo.XDirection;
import com.thalmic.myo.scanner.ScanActivity;

import org.ros.address.InetAddressFactory;
import org.ros.android.RosActivity;
import org.ros.node.NodeConfiguration;
import org.ros.node.NodeMainExecutor;

public class MainActivity extends RosActivity {

    public MainActivity() {
        // The RosActivity constructor configures the notification title and ticker
        // messages.
        super("Spiri Myo Commander", "Spiri Myo Commander");
    }

    MyoNode myo_node;


    // This code will be returned in onActivityResult() when the enable Bluetooth activity exits.
    private static final int REQUEST_ENABLE_BT = 1;

    private TextView mTextView;

    // Classes that inherit from AbstractDeviceListener can be used to receive events from Myo devices.
    // If you do not override an event, the default behavior is to do nothing.
    private DeviceListener mListener = new AbstractDeviceListener() {

        private Arm mArm = Arm.UNKNOWN;
        private XDirection mXDirection = XDirection.UNKNOWN;

        // onConnect() is called whenever a Myo has been connected.
        @Override
        public void onConnect(Myo myo, long timestamp) {
            // Set the text color of the text view to cyan when a Myo connects.
            mTextView.setText(getText(R.string.connected));
            mTextView.setTextColor(Color.CYAN);
        }

        // onDisconnect() is called whenever a Myo has been disconnected.
        @Override
        public void onDisconnect(Myo myo, long timestamp) {
            // Set the text color of the text view to red when a Myo disconnects.
            mTextView.setTextColor(Color.RED);
        }

        // onArmRecognized() is called whenever Myo has recognized a setup gesture after someone has put it on their
        // arm. This lets Myo know which arm it's on and which way it's facing.
        @Override
        public void onArmRecognized(Myo myo, long timestamp, Arm arm, XDirection xDirection) {
            mArm = arm;
            mXDirection = xDirection;
            mTextView.setText(R.string.got_arm);
        }

        // onArmLost() is called whenever Myo has detected that it was moved from a stable position on a person's arm after
        // it recognized the arm. Typically this happens when someone takes Myo off of their arm, but it can also happen
        // when Myo is moved around on the arm.
        @Override
        public void onArmLost(Myo myo, long timestamp) {
            mArm = Arm.UNKNOWN;
            mXDirection = XDirection.UNKNOWN;
            mTextView.setText(R.string.connected);
        }

        // onOrientationData() is called whenever a Myo provides its current orientation,
        // represented as a quaternion.
        @Override
        public void onOrientationData(Myo myo, long timestamp, Quaternion rotation) {
            // Calculate Euler angles (roll, pitch, and yaw) from the quaternion.
            float roll = (float) Math.toDegrees(Quaternion.roll(rotation));
            float pitch = (float) Math.toDegrees(Quaternion.pitch(rotation));
            float yaw = (float) Math.toDegrees(Quaternion.yaw(rotation));

            // Adjust roll and pitch for the orientation of the Myo on the arm.
            if (mXDirection == XDirection.TOWARD_ELBOW) {
                roll *= -1;
                pitch *= -1;
            }

            // Next, we apply a rotation to the text view using the roll, pitch, and yaw.
            // This tilts the text, and makes it hard to read
            //mTextView.setRotation(roll);
            //mTextView.setRotationX(pitch);
            //mTextView.setRotationY(yaw);

            myo_node.updateOrientation(roll, pitch, yaw);

            geometry_msgs.Twist twist = myo_node.getCurrentVelocityCommand();


            String twist_string = String.format("%s (vx:%f, vy:%f, vz:%f), yawrate:%f)", getCurrentHelpString(),
                    twist.getLinear().getX(), twist.getLinear().getY(), twist.getLinear().getZ(),
                    twist.getAngular().getZ()
            );
            mTextView.setText(twist_string);
        }

        public void onPose(Myo myo, long timestamp, Pose pose) {
            myo_node.updateGesture(pose);
        }

    };

    protected String getCurrentHelpString(){
        Pose pose = myo_node.getCurrentPose();
        String help_string = "";

        switch(pose){
            case UNKNOWN:
                help_string= getString(R.string.pose_unknown);
                break;
            case REST:
                help_string= getString(R.string.pose_rest);
                break;
            case FIST:
                help_string= getString(R.string.pose_fist);
                break;
            case THUMB_TO_PINKY:
                help_string= getString(R.string.pose_thumbtopinky);
                break;
            case FINGERS_SPREAD:
                help_string= getString(R.string.pose_fingersspread);
                break;
            case WAVE_IN:
                help_string = getString(R.string.pose_wavein);
                break;
            case WAVE_OUT:
                help_string = getString(R.string.pose_waveout);
                break;
            default:
                help_string = getString(R.string.pose_unknown);
                break;
        }

        return help_string;
    }



    @Override
    protected void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);
        setContentView(R.layout.activity_main);

        //mTextView = (TextView) findViewById(R.id.text);
        mTextView = (TextView) findViewById(R.id.text);

        // First, we initialize the Hub singleton with an application identifier.
        Hub hub = Hub.getInstance();
        if (!hub.init(this, getPackageName())) {
            // We can't do anything with the Myo device if the Hub can't be initialized, so exit.
            Toast.makeText(this, "Couldn't initialize Hub", Toast.LENGTH_SHORT).show();
            finish();
            return;
        }

        // Next, register for DeviceListener callbacks.
        hub.addListener(mListener);
    }

    @Override
    protected void onResume() {
        super.onResume();

        // If Bluetooth is not enabled, request to turn it on.
        if (!BluetoothAdapter.getDefaultAdapter().isEnabled()) {
            Intent enableBtIntent = new Intent(BluetoothAdapter.ACTION_REQUEST_ENABLE);
            startActivityForResult(enableBtIntent, REQUEST_ENABLE_BT);
        }
    }

    @Override
    protected void onDestroy() {
        super.onDestroy();
        // We don't want any callbacks when the Activity is gone, so unregister the listener.
        Hub.getInstance().removeListener(mListener);

        if (isFinishing()) {
            // The Activity is finishing, so shutdown the Hub. This will disconnect from the Myo.
            Hub.getInstance().shutdown();
        }
    }

    @Override
    protected void onActivityResult(int requestCode, int resultCode, Intent data) {
        // User chose not to enable Bluetooth, so exit.
        if (requestCode == REQUEST_ENABLE_BT && resultCode == Activity.RESULT_CANCELED) {
            finish();
            return;
        }
        super.onActivityResult(requestCode, resultCode, data);
    }

    @Override
    public boolean onCreateOptionsMenu(Menu menu) {
        super.onCreateOptionsMenu(menu);
        MenuInflater inflater = getMenuInflater();
        inflater.inflate(R.menu.menu_myo_pairing, menu);
        return true;
    }

    @Override
    public boolean onOptionsItemSelected(MenuItem item) {
        int id = item.getItemId();
        if (R.id.action_scan == id) {
            onScanActionSelected();
            return true;
        }
        return super.onOptionsItemSelected(item);
    }

    private void onScanActionSelected() {
        // Launch the ScanActivity to scan for Myos to connect to.
        Intent intent = new Intent(this, ScanActivity.class);
        startActivity(intent);
    }

    @Override
    protected void init(NodeMainExecutor nodeMainExecutor) {
        myo_node = new MyoNode();

        // Connect to a remote master based on the host address already provided by the dialogue
        NodeConfiguration nodeConfiguration =
                NodeConfiguration.newPublic(InetAddressFactory.newNonLoopback().getHostAddress());
        nodeConfiguration.setMasterUri(getMasterUri());

        nodeConfiguration.setMasterUri(getMasterUri());
        nodeMainExecutor.execute(myo_node, nodeConfiguration);
    }
}