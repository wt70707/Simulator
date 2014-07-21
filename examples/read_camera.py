from spiri_api import camera

spiri_camera=camera.camera_interface()

print spiri_camera.get_right_image()
print spiri_camera.get_left_image()
print spiri_camera.get_bottom_image()

#print spiri_camera.save_left_image('/home/rohan/Documents/sim.png')
