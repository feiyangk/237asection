#!/usr/bin/env python3

import numpy as np
import scipy.interpolate
from numpy import linalg

import rclpy
from asl_tb3_lib.math_utils import wrap_angle
from asl_tb3_lib.navigation import BaseNavigator, TrajectoryPlan
from asl_tb3_msgs.msg import TurtleBotControl, TurtleBotState


class AStar(object):
    """A* planner copied from HW1. Plotting helpers are omitted; this node only plans."""

    def __init__(self, statespace_lo, statespace_hi, x_init, x_goal, occupancy, resolution=1,
                 heuristic="l2", heuristic_weight=1.0):
        self.statespace_lo = statespace_lo
        self.statespace_hi = statespace_hi
        self.occupancy = occupancy
        self.resolution = resolution
        self.heuristic_type = heuristic
        self.heuristic_weight = heuristic_weight
        self.x_offset = x_init
        self.x_init = self.snap_to_grid(x_init)
        self.x_goal = self.snap_to_grid(x_goal)

        self.closed_set = set()
        self.open_set = set()

        self.est_cost_through = {}
        self.cost_to_arrive = {}
        self.came_from = {}

        self.open_set.add(self.x_init)
        self.cost_to_arrive[self.x_init] = 0
        self.est_cost_through[self.x_init] = self.heuristic(self.x_init)

        self.path = None

    def is_free(self, x):
        for d in range(len(x)):
            if x[d] < self.statespace_lo[d] or x[d] > self.statespace_hi[d]:
                return False
        return bool(self.occupancy.is_free(np.asarray(x, dtype=float)))

    def distance(self, x1, x2):
        return np.linalg.norm(np.array(x1) - np.array(x2))

    def heuristic(self, x):
        o = {"l1": 1, "l2": 2, "linf": np.inf}
        if self.heuristic_type not in o:
            raise ValueError("Unrecognized self.heuristic_type")
        return self.heuristic_weight * np.linalg.norm(
            np.array(x) - np.array(self.x_goal), ord=o[self.heuristic_type]
        )

    def snap_to_grid(self, x):
        return (
            self.resolution * round((x[0] - self.x_offset[0]) / self.resolution) + self.x_offset[0],
            self.resolution * round((x[1] - self.x_offset[1]) / self.resolution) + self.x_offset[1],
        )

    def get_neighbors(self, x):
        neighbors = []
        for dx, dy in [(1, 1), (0, 1), (-1, 1), (0, -1), (1, -1), (1, 0), (-1, 0), (-1, -1)]:
            n = self.snap_to_grid((x[0] + dx * self.resolution, x[1] + dy * self.resolution))
            if self.is_free(n):
                neighbors.append(n)
        return neighbors

    def find_best_est_cost_through(self):
        return min(self.open_set, key=lambda s: self.est_cost_through[s])

    def reconstruct_path(self):
        path = [self.x_goal]
        current = path[-1]
        while current != self.x_init:
            path.append(self.came_from[current])
            current = path[-1]
        return list(reversed(path))

    def solve(self):
        while self.open_set:
            current = self.find_best_est_cost_through()
            if current == self.x_goal:
                self.path = self.reconstruct_path()
                return True
            self.open_set.remove(current)
            self.closed_set.add(current)
            for nbr in self.get_neighbors(current):
                if nbr in self.closed_set:
                    continue
                tentative = self.cost_to_arrive[current] + self.distance(current, nbr)
                if nbr not in self.open_set:
                    self.open_set.add(nbr)
                elif tentative > self.cost_to_arrive[nbr]:
                    continue
                self.came_from[nbr] = current
                self.cost_to_arrive[nbr] = tentative
                self.est_cost_through[nbr] = tentative + self.heuristic(nbr)
        return False


class Navigator(BaseNavigator):
    def __init__(self) -> None:
        super().__init__()
        self.declare_parameter("kp", 2.0)
        self.declare_parameter("kpx", 2.0)
        self.declare_parameter("kpy", 2.0)
        self.declare_parameter("kdx", 2.0)
        self.declare_parameter("kdy", 2.0)
        self.declare_parameter("v_desired", 0.15)
        self.declare_parameter("spline_alpha", 0.05)

        self.V_PREV_THRES = 0.0001
        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

    @property
    def kp(self) -> float:
        return self.get_parameter("kp").value

    def compute_heading_control(
        self, state: TurtleBotState, goal: TurtleBotState
    ) -> TurtleBotControl:
        err = wrap_angle(goal.theta - state.theta)
        control = TurtleBotControl()
        control.omega = float(self.kp * err)
        return control

    def compute_trajectory_tracking_control(
        self, state: TurtleBotState, plan: TrajectoryPlan, t: float
    ) -> TurtleBotControl:
        dt = t - self.t_prev
        x, y, th = state.x, state.y, state.theta

        x_d = scipy.interpolate.splev(t, plan.path_x_spline, der=0)
        xd_d = scipy.interpolate.splev(t, plan.path_x_spline, der=1)
        xdd_d = scipy.interpolate.splev(t, plan.path_x_spline, der=2)
        y_d = scipy.interpolate.splev(t, plan.path_y_spline, der=0)
        yd_d = scipy.interpolate.splev(t, plan.path_y_spline, der=1)
        ydd_d = scipy.interpolate.splev(t, plan.path_y_spline, der=2)

        if abs(self.V_prev) < self.V_PREV_THRES:
            self.V_prev = self.V_PREV_THRES

        xd = self.V_prev * np.cos(th)
        yd = self.V_prev * np.sin(th)

        u = np.array([
            xdd_d + self.get_parameter("kpx").value * (x_d - x) + self.get_parameter("kdx").value * (xd_d - xd),
            ydd_d + self.get_parameter("kpy").value * (y_d - y) + self.get_parameter("kdy").value * (yd_d - yd),
        ])

        J = np.array([
            [np.cos(th), -self.V_prev * np.sin(th)],
            [np.sin(th), self.V_prev * np.cos(th)],
        ])
        a, om = linalg.solve(J, u)
        V = self.V_prev + a * dt

        self.t_prev = t
        self.V_prev = V
        self.om_prev = om

        control = TurtleBotControl()
        control.v = float(V)
        control.omega = float(om)
        return control

    def compute_trajectory_plan(
        self,
        state: TurtleBotState,
        goal: TurtleBotState,
        occupancy,
        resolution: float,
        horizon: float,
    ):
        astar = AStar(
            (state.x - horizon, state.y - horizon),
            (state.x + horizon, state.y + horizon),
            (state.x, state.y),
            (goal.x, goal.y),
            occupancy,
            resolution=resolution,
        )
        if (not astar.solve()) or astar.path is None or len(astar.path) < 4:
            return None

        self.V_prev = 0.0
        self.om_prev = 0.0
        self.t_prev = 0.0

        path = np.asarray(astar.path, dtype=float)
        v_desired = self.get_parameter("v_desired").value
        spline_alpha = self.get_parameter("spline_alpha").value
        ts = np.concatenate(
            ([0.0], np.cumsum(np.linalg.norm(np.diff(path, axis=0), axis=1)) / v_desired)
        )
        return TrajectoryPlan(
            path=path,
            path_x_spline=scipy.interpolate.splrep(ts, path[:, 0], s=spline_alpha),
            path_y_spline=scipy.interpolate.splrep(ts, path[:, 1], s=spline_alpha),
            duration=float(ts[-1]),
        )


if __name__ == "__main__":
    rclpy.init()
    node = Navigator()
    rclpy.spin(node)
    rclpy.shutdown()
