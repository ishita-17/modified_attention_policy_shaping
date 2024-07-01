#import rospy
#import armpy 

class Robot:

    def __init__(self):
        print("Initialising arm")
        #rospy.init_node("test", anonymous = True)
        #self.arm = armpy.kortex_arm.Arm()
        #self.arm = armpy.initialize("gen3_lite")
        #self.arm.home_arm()

    def reset(self, x, y, z):
       self.x1 = x
       self.x2 = y
       self.x3 = z

    def goto_cartesian_pose(self, x,y,z):
        #self.arm.goto_cartesian_pose_old([x,y,z,0,0,0,1], relative = True) 
        print("Moving arm")

    def home_arm(self):
        #self.arm.home_arm()
        print("Arm going home")

    def close_gripper(self):
        #self.arm.close_gripper()
        print("Arm closes gripper")

    def open_gripper(self):
        #self.arm.open_gripper()
        print("Arm opens gripper")
    

    