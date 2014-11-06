from spiri_api import api
#create a object for sending commands to Spiri
spiri=api.spiri_api_python()
# send goals with respect to start position
spiri.send_goal(0,0,1,True)
# send goals with respect to world
spiri.send_goal(0,0,1,False)
