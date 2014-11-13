##Using spiri_myo_commander##
###App startup###
1. Start a roscore on your laptop. Make sure you know your ROS_MASTER_URI.
2. Start the app with Android Studio
3. When the app launches, you will be prompted for your ROS_MASTER_URI. After you enter it, if you ```rostopic echo /cmd_vel``` on the laptop, you should see it publishing to /cmd_vel at around 3 hz.
4. Click the Scan menu in the top corner to search for a Myo. Select your Myo from the list, put it on, and click the back button.
5. Now perform the Myo [sync gesture](https://support.getmyo.com/hc/en-us/articles/200755509-How-to-perform-the-sync-gesture) and you are ready to go. Your current command will be displayed on the phone screen, and published to /cmd_vel. If you were to start the simulation now, the Myo would control the Spiri.

###Controls###
* Fingers Spread Gesture: Go up
* Thumb to pinky gesture: Go down
* Fist gesture and
    * Arm pitch: Forward and back speed
    * Arm roll: Side to side speed
    * Arm yaw: Yaw rate
* Anything else (rest, wave in, wave out, unknown): Do nothing (hover)


##Building and running spiri_myo_commander##
1. Get the [Android SDK](https://developer.android.com/sdk/index.html?hl=i), [Android Studio](https://developer.android.com/sdk/installing/studio.html), ros-hydro-rosjava-* (using apt), and the [Myo Android SDK](https://developer.thalmic.com/downloads)
2. Edit spiri_android/local.properties to point to your installation of the Android SDK.
3. Edit spiri_android/spiri_myo_commander/build.gradle to point to the myorepository directory
   of the Myo SDK.
4. Open Android Studio and import spiri_android as a project. After it is done importing the project (will take a minute), run Build -> Rebuild Project
5. On your Android device, enable USB debugging and the plug it in to your computer. Now you can run the app on your device through Android Studio.  
