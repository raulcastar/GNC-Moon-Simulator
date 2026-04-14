"""

Mobile robot motion planning sample with Dynamic Window Approach

author: Atsushi Sakai (@Atsushi_twi), Göktuğ Karakaşlı

"""

import math

import matplotlib.pyplot as plt
import numpy as np

show_animation = True


class Local_Trajectory_Controller():

    def dwa_control(self, x, robot_description, goal, ob):
        """
        Dynamic Window Approach control
        """
        dw = self.calc_dynamic_window(x, robot_description)

        u, trajectory = self.calc_control_and_trajectory(x, dw, robot_description, goal, ob)

        return u, trajectory


    def motion(self, x, u, dt):
        """
        motion model
        """

        x[2] += u[1] * dt
        x[0] += u[0] * math.cos(x[2]) * dt
        x[1] += u[0] * math.sin(x[2]) * dt
        x[3] = u[0]
        x[4] = u[1]

        return x


    def calc_dynamic_window(self, x, robot_description):
        """
        calculation dynamic window based on current state x
        """

        # Dynamic window from robot specification
        Vs = [robot_description.min_speed, robot_description.max_speed,
            -robot_description.max_yaw_rate, robot_description.max_yaw_rate]

        # Dynamic window from motion model
        Vd = [x[3] - robot_description.max_accel * robot_description.dt,
            x[3] + robot_description.max_accel * robot_description.dt,
            x[4] - robot_description.max_delta_yaw_rate * robot_description.dt,
            x[4] + robot_description.max_delta_yaw_rate * robot_description.dt]

        #  [v_min, v_max, yaw_rate_min, yaw_rate_max]
        dw = [max(Vs[0], Vd[0]), min(Vs[1], Vd[1]),
            max(Vs[2], Vd[2]), min(Vs[3], Vd[3])]

        return dw


    def predict_trajectory(self, x_init, v, y, robot_description):
        """
        predict trajectory with an input
        """

        x = np.array(x_init)
        trajectory = np.array(x)
        time = 0
        while time <= robot_description.predict_time:
            x = self.motion(x, [v, y], robot_description.dt)
            trajectory = np.vstack((trajectory, x))
            time += robot_description.dt

        return trajectory


    def calc_control_and_trajectory(self, x, dw, robot_description, goal, ob):
        """
        calculation final input with dynamic window
        """

        x_init = x[:]
        min_cost = float("inf")
        best_u = [0.0, 0.0]
        best_trajectory = np.array([x])

        # evaluate all trajectory with sampled input in dynamic window
        for v in np.arange(dw[0], dw[1], robot_description.v_resolution):
            for y in np.arange(dw[2], dw[3], robot_description.yaw_rate_resolution):

                trajectory = self.predict_trajectory(x_init, v, y, robot_description)
                # calc cost
                to_goal_cost = robot_description.to_goal_cost_gain * self.calc_to_goal_cost(trajectory, goal)
                speed_cost = robot_description.speed_cost_gain * (robot_description.max_speed - trajectory[-1, 3])
                ob_cost = robot_description.obstacle_cost_gain * self.calc_obstacle_cost(trajectory, ob, robot_description)

                final_cost = to_goal_cost + speed_cost + ob_cost

                # search minimum trajectory
                if min_cost >= final_cost:
                    min_cost = final_cost
                    best_u = [v, y]
                    best_trajectory = trajectory
                    if abs(best_u[0]) < robot_description.robot_stuck_flag_cons \
                            and abs(x[3]) < robot_description.robot_stuck_flag_cons:
                        # to ensure the robot do not get stuck in
                        # best v=0 m/s (in front of an obstacle) and
                        # best omega=0 rad/s (heading to the goal with
                        # angle difference of 0)
                        best_u[1] = -robot_description.max_delta_yaw_rate
        return best_u, best_trajectory


    def calc_obstacle_cost(self, trajectory, ob, robot_description):
        """
        calc obstacle cost inf: collision
        """
        ox = ob[:, 0]
        oy = ob[:, 1]
        dx = trajectory[:, 0] - ox[:, None]
        dy = trajectory[:, 1] - oy[:, None]
        r = np.hypot(dx, dy)

        if robot_description.robot_type == 1: # Rectangle
            yaw = trajectory[:, 2]
            rot = np.array([[np.cos(yaw), -np.sin(yaw)], [np.sin(yaw), np.cos(yaw)]])
            rot = np.transpose(rot, [2, 0, 1])
            local_ob = ob[:, None] - trajectory[:, 0:2]
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            local_ob = np.array([local_ob @ x for x in rot])
            local_ob = local_ob.reshape(-1, local_ob.shape[-1])
            upper_check = local_ob[:, 0] <= robot_description.robot_length / 2
            right_check = local_ob[:, 1] <= robot_description.robot_width / 2
            bottom_check = local_ob[:, 0] >= -robot_description.robot_length / 2
            left_check = local_ob[:, 1] >= -robot_description.robot_width / 2
            if (np.logical_and(np.logical_and(upper_check, right_check),
                            np.logical_and(bottom_check, left_check))).any():
                return float("Inf")
        elif robot_description.robot_type  == 0: # Circle
            if np.array(r <= robot_description.robot_radius).any():
                return float("Inf")

        min_r = np.min(r)
        return 1.0 / min_r  # OK


    def calc_to_goal_cost(self, trajectory, goal):
        """
            calc to goal cost with angle difference
        """

        dx = goal[0] - trajectory[-1, 0]
        dy = goal[1] - trajectory[-1, 1]
        error_angle = math.atan2(dy, dx)
        cost_angle = error_angle - trajectory[-1, 2]
        cost = abs(math.atan2(math.sin(cost_angle), math.cos(cost_angle)))

        return cost
    
    def get_lookahead_point(self, robot_pose, path, lookahead_dist = 2.0):
        """
        Finds a target point on the path that is 'lookahead_dist' away 
        from the robot's current position, moving forward along the path.
        """
        
        # Step 1: Find the index of the closest point on the path
        # We calculate the Euclidean distance from the robot to EVERY point
        dx = path[:, 0] - robot_pose[0]
        dy = path[:, 1] - robot_pose[1]
        distances = np.hypot(dx, dy)
        
        closest_idx = np.argmin(distances)
        
        # Step 2: Search forward from the closest point
        # We iterate starting from closest_idx to finding the first point
        # that is explicitly further away than our lookahead_dist.
        lookahead_idx = closest_idx
        
        while lookahead_idx < len(path) - 1:
            # Check distance from ROBOT to this candidate point
            dist_to_candidate = distances[lookahead_idx]
            
            if dist_to_candidate >= lookahead_dist:
                break
                
            lookahead_idx += 1
        
        # Return the coordinates of the found point
        return path[lookahead_idx]


    def plot_arrow(self, x, y, yaw, length=0.5, width=0.1):  # pragma: no cover
        plt.arrow(x, y, length * math.cos(yaw), length * math.sin(yaw),
                head_length=width, head_width=width)
        plt.plot(x, y)


    def plot_robot(self, x, y, yaw, robot_config):  # pragma: no cover
        if robot_config.robot_type == 1: #rectangle
            outline = np.array([[-robot_config.robot_length / 2, robot_config.robot_length / 2,
                                (robot_config.robot_length / 2), -robot_config.robot_length / 2,
                                -robot_config.robot_length / 2],
                                [robot_config.robot_width / 2, robot_config.robot_width / 2,
                                - robot_config.robot_width / 2, -robot_config.robot_width / 2,
                                robot_config.robot_width / 2]])
            Rot1 = np.array([[math.cos(yaw), math.sin(yaw)],
                            [-math.sin(yaw), math.cos(yaw)]])
            outline = (outline.T.dot(Rot1)).T
            outline[0, :] += x
            outline[1, :] += y
            plt.plot(np.array(outline[0, :]).flatten(),
                    np.array(outline[1, :]).flatten(), "-k")
        elif robot_config.robot_type == 0: # circle
            circle = plt.Circle((x, y), robot_config.robot_radius, color="b")
            plt.gcf().gca().add_artist(circle)
            out_x, out_y = (np.array([x, y]) +
                            np.array([np.cos(yaw), np.sin(yaw)]) * robot_config.robot_radius)
            plt.plot([x, out_x], [y, out_y], "-k")

