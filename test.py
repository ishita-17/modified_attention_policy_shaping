import rospy
import armpy
import IPython

def main():
    rospy.init_node("test", anonymous = True)
    arm = armpy.kortex_arm.Arm()
    arm = armpy.initialize("gen3_lite")
    arm.home_arm()
    print(arm.get_eef_pose())
    #arm.goto_cartesian_pose()
    START_JOINT_POSE = (1.469, 308.508, 80.089, 310.732, 321.826, 142.912)
    arm.goto_joint_pose(START_JOINT_POSE)
    IPython.embed()



if __name__ == "__main__":
    main()