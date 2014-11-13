To build the android app using catkin:

0) Get the Android SDK, android-studio, ros-hydro-rosjava-*, and the Myo Android SDK

1) edit spiri_android/local.properties to point to your installation of the android sdk

2) edit spiri_android/spiri_myo_commander/build.gradle to point to the myorepository directory
   of the Myo SDK.

3) Open android studio and import spiri_android as a project. It should build spiri_myo_commander
   as it is importing the project.

4) Plug in to the USB of your android device with USB debugging enabled, and then click run in Android Studio.


