from geometry_msgs.msg import TwistStamped
def initialize():
    global face_pos
    global sb_motor_0
    global sb_motor_publisher_0
    global default_speed
    face_pos=0
    default_speed=20