from multi_robomaster import multi_robot
from robomaster import robot
import numpy as np
import time

class cluster:
    def __init__(self, total_robots):
        #self.robots_sn = ['A', 'B', 'C', 'D', 'E']
        self.no_of_robots = total_robots
        self.robots_sn = ['3JKDH6C001J1LW', '3JKDH6C001W652']
        self.multirobots = multi_robot.MultiEP()
        self.multirobots.initialize()
        #self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]],[2, self.robots_sn[2]], [3, self.robots_sn[3]], [4, self.robots_sn[4]])
        self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]])
        # self.robot_all = self.multirobots.build_group([0, 1, 2, 3, 4])
        self.robot_all = self.multirobots.build_group([0, 1])
        self.locs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        # self.locs = np.array([[0, 0], [0, 0], [0, 0], [0, 0], [0, 0]], dtype=np.float16)
        self.yaws = [0, 0, 0, 0, 0]
        self.update_all(self.robot_all)
        self.L = np.array([[2, -1, -1, 0, 0], [0, 2, -1, -1, 0], [0, 0, 2, -1, -1], [-1, 0, 0, 2, -1], [-1, -1, 0, 0, 2]], dtype=np.float16)
        self.vx = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.omega = [0.0, 0.0, 0.0, 0.0, 0.0]
        self.err = np.zeros((self.no_of_robots, 2))
        self.err1 = -1*np.matmul(self.L, self.locs)
        self.v_max = 0.4
        self.tolerance = 0.2
        self.v_tolerance = 0.1*np.ones((1, self.no_of_robots))
        self.converged = False
        self.robots_reached = [False, False, False, False, False]
        self.x1 = 0
        self.x2 = 0
        self.y1 = 0
        self.y2 = 0
        self.conv_point = np.zeros((1, 2), dtype=np.float16)


    def printPoses(self):
        print(self.locs)
        # print(self.x1, self.y1, self.x2, self.y2)

    def position_updater(self, pos_info, rob_no):
        x, y, _ = pos_info
        self.locs[rob_no, 0] = x
        self.locs[rob_no, 1] = y

    #to be tested: works
    def yaw_updater(self, att_info, rob_no):
        yaw, _, __ = att_info
        self.yaws[rob_no] = yaw

    #to be tested; works well!
    def update_all(self, rob_grp):
        for i in range(self.no_of_robots):
            _ = rob_grp.get_robot(i)
            _.chassis.sub_position(0, 10, self.position_updater, i)
            _.chassis.sub_attitude(5, self.yaw_updater, i)

    def are_all_robots_reached(self):
        return self.robots_reached[0] and self.robots_reached[1] and self.robots_reached[2] and self.robots_reached[3] and self.robots_reached[4]

    # def move_fwd(self, rob_grp):
    #     rob = rob_grp.get_robot(0)
    #     rob.chassis.drive_speed(x=0.0, y=-1.0, z=0.0, timeout=None)
    #     while self.robots_reached[0] == False:
    #         pass
    #     rob.chassis.drive_speed(x=0, y=0, z=0)

    # def move_bkd(self, rob_grp):
    #     rob = rob_grp.get_robot(0)
    #     rob.chassis.drive_speed(x=0, y=1, z=0.0, timeout=None)
    #     time.sleep(5)
    #     self.robots_reached[0] = True
    #     rob.chassis.drive_speed(x=0, y=0, z=0)

    # def move_both(self, rob_grp):
    #     rob0 = rob_grp.get_robot(0)
    #     rob1 = rob_grp.get_robot(1)
    #     rob0.chassis.drive_speed(x=0, y=1, z=0)
    #     rob1.chassis.drive_speed(x=1, y=0, z=0)
    #     time.sleep(1)
    #     rob0.chassis.drive_speed(x=0, y=0, z=0)
    #     rob1.chassis.drive_speed(x=0, y=0, z=0)

        
    def move_toward_mid_point(self, rob_grp):
        goal_achieved = self.are_all_robots_reached()
        while not goal_achieved:
            for i in range(self.no_of_robots):
                robo = rob_grp.get_robot(i)
                curr_rob_pose = self.err1[i, :] #same as a in matlab code
                #linear velocity update:
                self.vx[i] = np.cos(self.yaws[i]) * curr_rob_pose[0] + np.sin(self.yaws[i]) * curr_rob_pose[1]
                if np.abs(self.vx[i]) > self.vmax:
                    self.vx[i] = np.sign(self.vx[i]) * self.vmax
                #angular velocity update:
                self.omega[i] = -np.sin(self.yaws[i])*curr_rob_pose[0] + np.cos(self.yaws[i])*curr_rob_pose[1]
                other_locs = self.locs
                other_locs[i,:] = []
                curr_rob_pose = self.locs[i,:]
                l2 = self.no_of_robots - 1
                for j in range(l2):
                    if np.linalg.norm(curr_rob_pose - other_locs[j,:]) < self.tolerance:
                        self.vx[i] = 0.0
                        self.omega[i] = 0.0
                        self.converged = True
                        self.conv_point = curr_rob_pose
                        self.robots_reached[i] = True

                robo.chassis.drive_speed(x=self.vx[i], y=0.0, z=self.omega[i])

            self.err1 = -1*np.matmul(self.L, self.locs)
            for i in range(self.no_of_robots):
                if self.converged == True:
                    self.err1[i, :] = -1*self.locs[i,:] + self.conv_point

            goal_achieved = self.are_all_robots_reached()
        
        print("GOAL ACHIEVED!")
                

        
# robo0 = rovers.robot0.get_robot(0)
# robo1 = rovers.robot1.get_robot(0)

# robo0.chassis.sub_position(freq=10, callback=rovers.position_updater0)
# robo1.chassis.sub_position(freq=10, callback=rovers.position_updater1)
# rovers.robot_all.chassis.move(1, 0, 0, 2, 90).wait_for_completed()
try:
    
    rovers = cluster(2)
    rovers.multirobots.run([rovers.robot_all, rovers.move_toward_mid_point])
except KeyboardInterrupt:
    pass