from multi_robomaster import multi_robot
import numpy as np
import time

formation_specs = {
    'I' : np.array(
        [
            [0, 0],
            [1, 1],
            [4, 0],
            [3, 1],
            [4, 2],
            [0, 2]
        ]
    ),

    'M' : np.array(
        [
            [0, 0],
            [2.5, 1],
            [4, 0],
            [2.5, 2],
            [4, 3],
            [0, 3]
        ]
    ),

    'A': np.array(
        [
            [0, 0],
            [2, 1],
            [4, 2],
            [2, 2],
            [2, 3],
            [0, 4]
        ]
    ),

    'S': np.array(
        [
            [0.5, 0],
            [3, 0.5],
            [3.5, 1.5],
            [2, 1],
            [1, 2],
            [0, 1]
        ]
    )
}


class formations:
    def __init__(self, total_robots):
        self.no_of_robots = total_robots
        

        ### manual config
        self.robots_sn = ['3JKDH6C001W652', '3JKDH6C001J1LW',  '3JKDH6C0019B1H', '3JKDH6C0014BKJ','3JKDH6C001G351', '3JKDH6C001P7PZ']
        

        self.multirobots = multi_robot.MultiEP()
        self.multirobots.initialize()
        

        ### manual config
        self.numbers = self.multirobots.number_id_by_sn([0, self.robots_sn[0]], [1, self.robots_sn[1]], [2, self.robots_sn[2]], [3, self.robots_sn[3]], [4, self.robots_sn[4]], [5, self.robots_sn[5]])
        self.robot_all = self.multirobots.build_group([0, 1, 2, 3, 4, 5])


        self.origins = np.array(
            [
                [0, -1.35],
                [0, -0.9],
                [0, -0.45],
                [0, 0],
                [0, 0.45],
                [0, 0.9]
            ]
        )
        
        self.locs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.yaws = [0.0 for _ in range(self.no_of_robots)]
        self.update_all(self.robot_all)
        self.errs = np.zeros((self.no_of_robots, 2), dtype=np.float16)
        self.vx = [0.0 for _ in range(self.no_of_robots)]
        self.omegas = [0.0 for _ in range(self.no_of_robots)]
        self.v_max = 0.2
        self.v_min = 0.01
        self.tolerance = 0.3

    def update_errors(self, shape):
        loc_set = 0.5 * formation_specs[shape]

        for i in range(self.no_of_robots):
            self.errs[i,:] = [0, 0]
            for j in range(self.no_of_robots):
                if i == j:
                    continue

                self.errs[i,:] = self.errs[i,:] + ((self.locs[i,:] - self.locs[j, :]) - (loc_set[i, :] - loc_set[j, :]))
            self.errs[i, :] = -1 * self.errs[i, :]

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
        print('making {}'.format(shape))
        self.update_errors(shape)
        
        while not goal_achieved:
            for i in range(self.no_of_robots):
                robo = rob_grp.get_robot(i)
                err = self.errs[i, :]
                if np.linalg.norm(err) > self.tolerance:
                    self.vx[i] = np.cos(np.deg2rad(self.yaws[i])) * err[0] + np.sin(np.deg2rad(self.yaws[i])) * err[1]

                    if abs(self.vx[i]) > self.v_max:
                        self.vx[i] = np.sign(self.vx[i]) * self.v_max
                    
                    self.omegas[i] = -np.sin(np.deg2rad(self.yaws[i])) * err[0] + np.cos(np.deg2rad(self.yaws[i])) * err[1]
                    self.omegas[i] = self.omegas[i] * (180 / np.pi)
                else:
                    robots_reached[i] = True
                    self.vx[i] = 0
                    self.omegas[i] = 0
                # print('sending to robot {}'.format(i))
                # print('vx {}'.format(self.vx))
                # print('err {}'.format(self.errs))
                # print('yaws {}'.format(self.yaws))
                robo.chassis.drive_speed(x = self.vx[i], y = 0.0, z = self.omegas[i])
            self.update_errors(shape)
            goal_achieved = all(robots_reached)

        # #fix heading so all robots face the same direction
        # while sum(self.yaws) < 10:
        #     for i in range(self.no_of_robots):
        #         robo = rob_grp.get_robot(i)
        #         u = 0.75 * (0 - self.yaws[i])
        #         robo.chassis.drive_speed(x = 0, y = 0, z = u)

        #after heading is fixed, stop robots altogether
        for i in range(self.no_of_robots):
            robo = rob_grp.get_robot(i)
            robo.chassis.drive_speed(x = 0, y = 0, z = 0)

        print('formation {} complete'.format(shape))

rovers = formations(6)
rovers.create_formation(rovers.robot_all, 'I')
time.sleep(3)
rovers.create_formation(rovers.robot_all, 'M')
time.sleep(3)
rovers.create_formation(rovers.robot_all, 'A')
time.sleep(3)
rovers.create_formation(rovers.robot_all, 'S')
