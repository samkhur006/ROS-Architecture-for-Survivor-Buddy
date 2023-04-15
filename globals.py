from geometry_msgs.msg import TwistStamped
def initialize():
    global count
    global distance
    global ld
    global rd
    global cur
    global an_cur
    global last_time
    global confused
    global last_left
    global last_right
    global sound
    count = 0
    distance=-1
    ld=-1
    rd=-1
    cur=0
    an_cur=0
    last_time=-1
    confused=True
    last_left=-1
    last_right=-1
    sound=False
    twist = TwistStamped()