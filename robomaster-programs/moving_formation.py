from multi_robomaster import multi_robot
import numpy as np
import matplotlib.pyplot as plt
import time

formation_specs = {'square': np.array([
                    [0, 0],
                    [0.8, 0],
                    [1.6, 0],
                    [0, 0]]
                ),
            'triangle': np.array([
                    [0, 0],
                    [1.5, 0.5*np.sqrt(3)],
                    [0, np.sqrt(3)],
                    [0.5, 0.5*np.sqrt(3)]
                    ]
                ),
            'trapezium': np.array([
                    [1, 0.5],
                    [1, 1.5],
                    [0, 2],
                    [0, 0]]
                ),
            'line': np.array([
                    [0, 0.5],
                    [0, 1],
                    [0, 1.5],
                    [0, 0]]
            )
              }


L = np.array(
            [
                [3, -1, -1, -1],
                [-1, 3, -1, 3],
                [-1, -1, 3, -1],
                [-1, -1, -1, 3]
            ]
        )

goal_loc = np.array(
    [
        [5,0.45],
        [10,0.45],
        [15,0.45],
        [20,0.45],
        [25, 0.45],
        [30, 0.45],
        [31.5, 0.45]
    ]
)

dt = 0.05

class formations:
    def __init__(self, total_robots):
        self.no_of_robots = total_robots
        self.robots_sn = ['3JKDH6C001W652', '3JKDH6C001J1LW',  '3JKDH6C0014BKJ']
        self.multirobots = multi_robot.MultiEP()
        self.multirobots.initialize()
        self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]], [2, self.robots_sn[2]])
        self.origins = np.array(
            [
                [0.0, 0.0],
                [0.0, 0.45],
                [0.0, 0.9],
                [0.0, 0.45]
            ]
        )
        self.robot_all = self.multirobots.build_group([0, 1, 2])
        self.locs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.yaws = [0.0, 0.0, 0.0, 0.0]
        self.update_all(self.robot_all)
        self.errs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.vx = [0.0, 0.0, 0.0, 0.0]
        self.omegas = [0.0, 0.0, 0.0, 0.0]
        self.v_max = 0.4
        self.v_min = 0.01
        self.count = 0
        self.tolerance = 0.08
        self.locs[3, :] = self.origins[3,:]
        

    def update_errors(self, shape):
        loc_set = 0.5 * formation_specs[shape]

        #only updating error for 3 actual robots
        for i in range(3):
            self.errs[i,:] = [0, 0]
            for j in range(self.no_of_robots):
                if i == j:
                    continue

                self.errs[i,:] = self.errs[i,:] + ((self.locs[i,:] - self.locs[j, :]) - (loc_set[i, :] - loc_set[j, :]))
            self.errs[i, :] = -1 * self.errs[i, :]

        #robot 4 is the ghost, it's error will be calculated using go to goal
        self.errs[3,0] = -5.5*(self.locs[3,0] - goal_loc[self.count,0])
        self.errs[3,1] = -5*(self.locs[3,1] - goal_loc[self.count,1])



    def position_updater(self, pos_info, rob_no):
        x, y, _ = pos_info
        self.locs[rob_no, 0] = x + self.origins[rob_no, 0]
        self.locs[rob_no, 1] = y + self.origins[rob_no, 1]

    def yaw_updater(self, att_info, rob_no):
        yaw, _, __ = att_info
        self.yaws[rob_no] = yaw

    def update_all(self, rob_grp):
        for i in range(self.no_of_robots - 1):
            _ = rob_grp.get_robot(i)
            _.chassis.sub_position(0, 10, self.position_updater, i)
            _.chassis.sub_attitude(10, self.yaw_updater, i)
    
    def create_formation(self, rob_grp, shape):
        robots_reached = [False for _ in range(self.no_of_robots)]
        goal_achieved = all(robots_reached)
        self.update_errors(shape)
        # epoch = 0
        while not goal_achieved:
            for i in range(self.no_of_robots):
                # if self.robots_reached[i]:
                #     continue
                if i != 3:
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
                    self.omegas[i] = self.omegas[i] * (180 / np.pi) 
                    # print('omegas', self.omegas)
                    # if abs(self.omegas[i]) > self.v_max:
                    #     self.omegas[i] = np.sign(self.omegas[i]) * self.v_max
                else:
                    robots_reached[i] = True
                    self.vx[i] = 0
                    self.omegas[i] = 0

                    # print('robot {} reached'.format(i + 1))
                if robots_reached[3] and self.count != 5:
                    self.count += 1
                    self.count = self.count % 6
                    robots_reached[3] = False
        
                if i != 3:
                    robo.chassis.drive_speed(x=self.vx[i], y =0.0, z = self.omegas[i])
                else:
                    #since this is a ghost robot, we will simulate its movement and pose ## ONLY FOR GHOST ROBOT ##
                    if abs(self.vx[i]) > 0.4:
                        self.vx[i] = np.sign(self.vx[i]) * 0.4 
                    self.yaws[i] = self.yaws[i] + dt *self.omegas[i]
                    self.locs[i, 0] = self.locs[i, 0] + dt * self.vx[i]*np.cos(np.deg2rad(self.yaws[i])) 
                    self.locs[i, 1] = self.locs[i, 1] + dt * self.vx[i]*np.sin(np.deg2rad(self.yaws[i])) 
                    print(self.locs[i, :])

            # if not epoch % 1000:
            #     plt.draw()
            #     plt.pause(0.000003)
            #     plt.plot(self.locs[0, 0], self.locs[0, 1], '.b', linewidth='5')
            #     plt.plot(self.locs[1, 0], self.locs[1, 1], '.r', linewidth='5')
            #     plt.plot(self.locs[2, 0], self.locs[2, 1], '.g', linewidth='5')
            #     plt.plot(self.locs[3, 0], self.locs[3, 1], '.k', linewidth='5')
            #     plt.show()
            # epoch += 1
            self.update_errors(shape)
            # if not any(self.vx) and not any(self.omegas):
            #     print('mission accomplished')
            #     break
            goal_achieved = all(robots_reached)
            time.sleep(dt)
        
        #fix heading so all robots face the same direction
        # while sum(self.yaws[0:2]) < 15:
        #     for i in range(4):
        #         robo = rob_grp.get_robot(i)
        #         u = 0.75 * (0 - self.yaws[i])
        #         robo.chassis.drive_speed(x = 0, y = 0, z = u)
        #after heading is fixed, stop robot altogether
        for i in range(3):
            robo = rob_grp.get_robot(i)
            robo.chassis.drive_speed(x = 0, y = 0, z = 0)


        print('formation {} complete.'.format(shape))

try:
    rovers = formations(4)
    rovers.create_formation(rovers.robot_all, 'triangle')
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