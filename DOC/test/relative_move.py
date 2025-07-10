from cri_lib import CRIController

# CRIController is the main interface for controlling the iRC
controller = CRIController()

#connect to default iRC IP
#controller.connect("192.168.3.11")
if not controller.connect("192.168.3.11", 3920):
    print("Unable to connect")
    quit()

#acquire active control.
controller.set_active_control(True)

print("enable")
#enable motors
controller.enable()

print("waiting")
#wait until kinematics are ready to move
controller.wait_for_kinematics_ready(10)

controller.set_override(100.0)

print("move")
#relative move x,y,z +20mm and back
controller.move_base_relative(20.0, 20.0, 20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, wait_move_finished=True, move_finished_timeout= 1000)
controller.move_base_relative(-20.0, -20.0, -20.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 10.0, wait_move_finished=True, move_finished_timeout= 1000)



#Disable motors and disconnect
controller.disable()
controller.close()
