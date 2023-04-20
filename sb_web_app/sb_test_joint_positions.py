import rospy
from geometry_msgs.msg import TwistStamped

twist = TwistStamped()

# helper function to convert input from rads to degrees


def degrees(x):
    return x * 180 / 3.141592653589793

# main function


def main():
    print("Joint positions are in degrees and input needs to be in the following order: ")
    print("joint1 joint2 joint3 joint4\n")
    print("Enter 'q' or 'quit' or 'exit' to quit\n")
    pub = rospy.Publisher('/sb_cmd_state', TwistStamped, queue_size=10)
    # pub = rospy.Publisher('/sb_0_cmd_state', TwistStamped, queue_size=10)
    rospy.init_node('test_joint', anonymous=True)
    rate = rospy.Rate(10)  # 10hz
    while not rospy.is_shutdown():
        in_val = input("\nEnter joint positions: ")
        if in_val.lower() == 'q' or in_val.lower() == 'quit' or in_val.lower() == 'exit':
            break
        if len(in_val.split()) != 4:
            print("Invalid input. Please enter 4 joint positions")
            continue
        goalPos = [float(x) for x in in_val.split()]
        global twist
        twist.twist.linear.x = -goalPos[0]
        twist.twist.linear.y = -goalPos[1]
        twist.twist.linear.z = goalPos[2]
        twist.twist.angular.x = -goalPos[3]
        print("Sending joint positions: ", goalPos)
        pub.publish(twist)
        rate.sleep()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
