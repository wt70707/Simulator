from planner_request import get_state
#create a object for sending commands to Spiri
spiri=get_state.Staterobot()
# send goals with respect to start position
spiri.send_goal_relative(0,0,1)
# send goals with respect to world
#spiri.send_goal(0,0,1)
