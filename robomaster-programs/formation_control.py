from multi_robomaster import multi_robot
from robomaster import robot
import numpy as np
import time

class formations:
    def __init__(self, total_robots):
        self.no_of_robots = total_robots
        self.robots_sn = ['3JKDH6C001W652', '3JKDH6C001P7PZ', '3JKDH6C0019B1H', '3JKDH6C001J1LW']
        self.multirobots = multi_robot.MultiEP()
        self.multirobots.initialize()
        self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]], [2, self.robots_sn[2]], [3, self.robots_sn[3]])
        self.origins = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.45],
                [0.0, 1.35],
                [0.0, 0.9]
            ]
        )
        
        self.loc_set = np.array(
            [           
                [0, 0],
                [1, 1],
                [0, 1]
            ]
            )
        
        self.loc_set = np.array(
        [
                [1, 0],
                [1, 1],
                [1, 2],
                [0, 0]
            ]
        )
        self.robot_all = self.multirobots.build_group([0, 1, 2, 3])
        self.locs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.yaws = [0.0, 0.0, 0.0, 0.0]
        self.update_all(self.robot_all)
        self.errs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.vx = [0.0, 0.0, 0.0, 0.0]
        self.omegas = [0.0, 0.0, 0.0, 0.0]
        self.v_max = 0.2
        self.v_min = 0.01
        self.tolerance = 0.3
        self.robots_reached = [False for _ in range(self.no_of_robots)]
        #initial update of errors
        self.update_errors()

    def update_errors(self):
        if self.no_of_robots == 3:
            self.errs[0,:] = -1*(((self.locs[0,:] - self.locs[1,:]) - (self.loc_set[0,:] - self.loc_set[1,:])) + ((self.locs[0,:] - self.locs[2,:]) - (self.loc_set[0,:] - self.loc_set[2,:])))
            self.errs[1,:] = -1*(((self.locs[1,:] - self.locs[0,:]) - (self.loc_set[1,:] - self.loc_set[0,:])) + ((self.locs[1,:] - self.locs[2,:]) - (self.loc_set[1,:] - self.loc_set[2,:])))
            self.errs[2,:] = -1*(((self.locs[2,:] - self.locs[1,:]) - (self.loc_set[2,:] - self.loc_set[1,:])) + ((self.locs[2,:] - self.locs[0,:]) - (self.loc_set[2,:] - self.loc_set[0,:])))
        if self.no_of_robots == 4:
            self.errs[0,:] = -1*(((self.locs[0,:] - self.locs[1,:]) - (self.loc_set[0,:] - self.loc_set[1,:])) + ((self.locs[0,:] - self.locs[3,:]) - (self.loc_set[0,:] - self.loc_set[3,:])))
            self.errs[1,:] = -1*(((self.locs[1,:] - self.locs[0,:]) - (self.loc_set[1,:] - self.loc_set[0,:])) + ((self.locs[1,:] - self.locs[2,:]) - (self.loc_set[1,:] - self.loc_set[2,:])))
            self.errs[2,:] = -1*(((self.locs[2,:] - self.locs[1,:]) - (self.loc_set[2,:] - self.loc_set[1,:])) + ((self.locs[2,:] - self.locs[3,:]) - (self.loc_set[2,:] - self.loc_set[3,:])))
            self.errs[3,:] = -1*(((self.locs[3,:] - self.locs[0,:]) - (self.loc_set[3,:] - self.loc_set[0,:])) + ((self.locs[3,:] - self.locs[2,:]) - (self.loc_set[3,:] - self.loc_set[2,:])))


    def position_updater(self, pos_info, rob_no):
        x, y, _ = pos_info
        self.locs[rob_no, 0] = x + self.origins[rob_no, 0]
        self.locs[rob_no, 1] = y + self.origins[rob_no, 1]

    def yaw_updater(self, att_info, rob_no):
        yaw, _, __ = att_info
        self.yaws[rob_no] = yaw

    def update_all(self, rob_grp):
        for i in range(self.no_of_robots):
            _ = rob_grp.get_robot(i)
            _.chassis.sub_position(0, 10, self.position_updater, i)
            _.chassis.sub_attitude(10, self.yaw_updater, i)

    def are_all_robots_reached(self):
        return all(self.robots_reached)
    
    def create_formation(self, rob_grp):
        goal_achieved = self.are_all_robots_reached()
        self.update_errors()
        while not goal_achieved:
            for i in range(self.no_of_robots):
                # if self.robots_reached[i]:
                #     continue
                robo = rob_grp.get_robot(i)
                err = self.errs[i, :]
                if np.linalg.norm(err) > self.tolerance:
                    #linear velocity update
                    self.vx[i] = np.cos(np.deg2rad(self.yaws[i])) * err[0] + np.sin(np.deg2rad(self.yaws[i])) * err[1]
                    print('vx', self.vx)
                    
                    if abs(self.vx[i]) > self.v_max:
                        self.vx[i] = np.sign(self.vx[i]) * self.v_max
                    # if abs(self.vx[i]) < self.v_min:
                    #     self.vx[i] = 0

                    #angular velocity update
                    self.omegas[i] = -np.sin(np.deg2rad(self.yaws[i])) * err[0] + np.cos(np.deg2rad(self.yaws[i]))*err[1]
                    self.omegas[i] = self.omegas[i]* (180 / np.pi) 
                    print('omegas', self.omegas)
                    # if abs(self.omegas[i]) > self.v_max:
                    #     self.omegas[i] = np.sign(self.omegas[i]) * self.v_max
                else:
                    self.robots_reached[i] = True
                    self.vx[i] = 0
                    self.omegas[i] = 0
                    # print('robot {} reached'.format(i + 1))
                
                robo.chassis.drive_speed(x=self.vx[i], y =0.0, z = self.omegas[i])

            self.update_errors()
            # if not any(self.vx) and not any(self.omegas):
            #     print('mission accomplished')
            #     break
            goal_achieved = self.are_all_robots_reached()

try:
    rovers = formations(4)
    # rovers.multirobots.run([rovers.robot_all, rovers.create_formation])
    rovers.create_formation(rovers.robot_all)
    # while True:
    #     print(rovers.locs)
    #     print(rovers.yaws)
    #     time.sleep(2)
except KeyboardInterrupt:
    pass