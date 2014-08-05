#!/usr/bin/env python
from spiri_api import libspiri_api_python as s
import numpy as np
class spiri_camera_interface():
  def __init__(self):
    self.camera_api=s.Staterobot()
  def get_left_image(self):
    
    image=(np.fromstring(self.camera_api.get_left_image(),dtype=np.uint8)).reshape(320,240,3)
    return image
  
  def get_right_image(self):
    
    image=(np.fromstring(self.camera_api.get_right_image(),dtype=np.uint8)).reshape(320,240,3)
    return image
  
  def get_bottom_image(self):
    
    image=(np.fromstring(self.camera_api.get_bottom_image(),dtype=np.uint8)).reshape(640,480,3)
    return image
