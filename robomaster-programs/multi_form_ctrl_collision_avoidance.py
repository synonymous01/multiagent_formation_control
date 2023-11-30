from multi_robomaster import multi_robot
import numpy as np
import time

formation_specs = {'square': np.array([
                    [1, 0],
                    [1, 1],
                    [0, 1],
                    [0, 0]]
                ),
            'triangle': np.array([
                    [0.5, 0.5*np.sqrt(3)],
                    [1.5, 0.5*np.sqrt(3)],
                    [0, np.sqrt(3)],
                    [0, 0]]
                ),
            'trapezium': np.array([
                    [0, 2],
                    [1, 0.5],
                    [1, 1.5],
                    [0, 0]]
                ),
            'line': np.array([
                    [0, 1.5],
                    [0, 1],
                    [0, 0.5],
                    [0, 0]]
            ),
            'triangle without centroid': np.array(
                [
                    [0, 0],
                    [0, 2],
                    [np.sqrt(3), 1]
                ]
            )
              }


class formations:
    def __init__(self, total_robots):
        self.no_of_robots = total_robots
        self.robots_sn = ['3JKDH6C001W652', '3JKDH6C001J1LW',  '3JKDH6C0019B1H', '3JKDH6C001P7PZ']
        self.multirobots = multi_robot.MultiEP()
        self.multirobots.initialize()
        self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]], [2, self.robots_sn[2]], [3, self.robots_sn[3]])
        self.origins = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.45],
                [0.0, 0.9],
                [0.0, 2.7]
            ]
        )
        self.robot_all = self.multirobots.build_group([0, 1, 2])
        self.locs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.yaws = [0.0, 0.0, 0.0, 0.0]
        self.errs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.vx = [0.0, 0.0, 0.0, 0.0]
        self.omegas = [0.0, 0.0, 0.0, 0.0]
        self.v_max = 0.2
        self.v_min = 0.01
        self.tolerance = 0.2
        # self.weights = np.zeros((self.no_of_robots, self.no_of_robots))
        self.weights = np.zeros((self.no_of_robots))
        self.update_all(self.robot_all)
        self.update_dists()
        

    def update_errors(self, shape):
        loc_set = formation_specs[shape]
        # self.errs[0,:] = -1 * (((self.locs[0,:] - self.locs[1,:]) - (loc_set[0,:] - loc_set[1,:])) + ((self.locs[0,:] - self.locs[3,:]) - (loc_set[0,:] - loc_set[3,:])))
        # self.errs[1,:] = -1 * (((self.locs[1,:] - self.locs[0,:]) - (loc_set[1,:] - loc_set[0,:])) + ((self.locs[1,:] - self.locs[2,:]) - (loc_set[1,:] - loc_set[2,:])))
        # self.errs[2,:] = -1 * (((self.locs[2,:] - self.locs[1,:]) - (loc_set[2,:] - loc_set[1,:])) + ((self.locs[2,:] - self.locs[3,:]) - (loc_set[2,:] - loc_set[3,:])))
        # self.errs[3,:] = -1 * (((self.locs[3,:] - self.locs[0,:]) - (loc_set[3,:] - loc_set[0,:])) + ((self.locs[3,:] - self.locs[2,:]) - (loc_set[3,:] - loc_set[2,:])))
        
        self.errs[0,:] = -1*(((self.locs[0,:] - self.locs[1,:]) - (loc_set[0,:] - loc_set[1,:])) + ((self.locs[0,:] - self.locs[2,:]) - (loc_set[0,:] - loc_set[2,:])))
        self.errs[1,:] = -1*(((self.locs[1,:] - self.locs[0,:]) - (loc_set[1,:] - loc_set[0,:])) + ((self.locs[1,:] - self.locs[2,:]) - (loc_set[1,:] - loc_set[2,:])))
        self.errs[2,:] = -1*(((self.locs[2,:] - self.locs[1,:]) - (loc_set[2,:] - loc_set[1,:])) + ((self.locs[2,:] - self.locs[0,:]) - (loc_set[2,:] - loc_set[0,:])))
        
        for k in range(3):
            self.errs[k, :] = np.multiply(self.errs[k, :], self.weights[k]*2)
        # self.errs = -1*np.matmul(L, self.locs - loc_set)

    def update_dists(self):
        collision_threshold = 0.4
        dists = [0, 0, 0, 0]
        dists[0] = np.linalg.norm(self.locs[0, :] - self.locs[1, :])
        dists[1] = np.linalg.norm(self.locs[1,:] - self.locs[2, :])
        dists[2] = np.linalg.norm(self.locs[2, :] - self.locs[0, :])

        self.weights[0] = (dists[0] + dists[2]) - collision_threshold
        self.weights[1] = (dists[0] + dists[1]) - collision_threshold
        self.weights[2] = (dists[2] + dists[1]) - collision_threshold
        # for i in range(self.no_of_robots):
        #     for j in range(self.no_of_robots):
        #         if i == j:
        #             continue
                
        #         self.weights[i, j] = np.linalg.norm(self.locs[i, :] - self.locs[j, :]) - collision_threshold
        #         if self.weights[i, j] < 0.1:
        #             self.weights[i,j] = 0.1


        #dists[0] = np.linalg.norm

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
    
    def create_formation(self, rob_grp, shape):
        robots_reached = [False for _ in range(self.no_of_robots)]
        goal_achieved = all(robots_reached)
        self.update_dists()
        self.update_errors(shape)
        while not goal_achieved:
        # while True:
            print(self.weights)
            for i in range(self.no_of_robots):
                # if self.robots_reached[i]:
                #     continue
                robo = rob_grp.get_robot(i)
                err = self.errs[i , :]
                if np.linalg.norm(err) > self.tolerance:
                    #linear velocity update
                    self.vx[i] = np.cos(np.deg2rad(self.yaws[i])) * err[0] + np.sin(np.deg2rad(self.yaws[i])) * err[1]
                    # print('vx', self.vx)
                    
                    if abs(self.vx[i]) > self.v_max:
                        self.vx[i] = np.sign(self.vx[i]) * self.v_max
                    # if abs(self.vx[i]) < self.v_min:
                    #     self.vx[i] = 0

                    #angular velocity update
                    self.omegas[i] = -np.sin(np.deg2rad(self.yaws[i])) * err[0] + np.cos(np.deg2rad(self.yaws[i]))*err[1]
                    self.omegas[i] = self.omegas[i]* (180 / np.pi) 
                    # print('omegas', self.omegas)
                    # if abs(self.omegas[i]) > self.v_max:
                    #     self.omegas[i] = np.sign(self.omegas[i]) * self.v_max
                else:
                    robots_reached[i] = True
                    self.vx[i] = 0
                    self.omegas[i] = 0
                    # print('robot {} reached'.format(i + 1))
                
                robo.chassis.drive_speed(x=self.vx[i], y =0.0, z = self.omegas[i], timeout = 1)

            self.update_dists()
            self.update_errors(shape)
            # if not any(self.vx) and not any(self.omegas):
            #     print('mission accomplished')
            #     break
            goal_achieved = all(robots_reached)
        
        #fix heading so all robots face the same direction
        print('fixing heading')
        while sum(self.yaws) > 10:
            for i in range(4):
                robo = rob_grp.get_robot(i)
                u = 0.75 * (0 - self.yaws[i])
                robo.chassis.drive_speed(x = 0, y = 0, z = u, timeout= 1)
        #after heading is fixed, stop robot altogether
        print('all robots stopped')
        for _ in range(10):
            for i in range(4):
                robo = rob_grp.get_robot(i)
                robo.chassis.drive_speed(x = 0.0, y = 0.0, z = 0.0, timeout=1)


        print('formation {} complete.'.format(shape))

try:
    rovers = formations(3)
    rovers.create_formation(rovers.robot_all, 'triangle without centroid')
    # time.sleep(4)
    # rovers.create_formation(rovers.robot_all, 'triangle')
    # time.sleep(4)
    # rovers.create_formation(rovers.robot_all, 'trapezium')
    # time.sleep(4)
    # rovers.create_formation(rovers.robot_all, 'line')
    # rovers.multirobots.run([rovers.robot_all, rovers.create_formation])
    # while True:
    #     print(rovers.locs)
    #     print(rovers.yaws)
    #     time.sleep(2)
except KeyboardInterrupt:
    pass