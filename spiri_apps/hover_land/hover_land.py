#!/usr/bin/env python
from spiri_api import spiri_api_py

spiri=spiri_api_py.Staterobot()
state=spiri.get_state()
s=raw_input('Do you want to land[l] or hover[h]')
if s=='l':
  spiri.send_vel([0,0,-state[2]])
  spiri.send_vel([0,0,0])

elif s=='h':
  if state[2]<1.0:
    spiri.send_vel([0,0,1])
    spiri.send_vel([0,0,0]) 

else:
  print "Invalid command"
  
  
#print state
# Hover
'''

  
'''

# Landing


  