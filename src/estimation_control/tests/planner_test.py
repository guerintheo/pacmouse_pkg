import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

from pacmouse_pkg.src.utils.maze import Maze2
from pacmouse_pkg.src.estimation_control.planner import Planner


class PlannerTest:
    
    def __init__(self):
        self.m = Maze2(4,4)
        #self.m.generate_random_maze()
        self.m.set_wall_between(0, 1, 1)
        self.m.set_wall_between(4, 5, 1)
        self.m.set_wall_between(8, 12, 1)
        self.m.set_wall_between(9, 13, 1)
        self.m.set_wall_between(5, 6, 1)
        self.m.set_wall_between(6, 10, 1)
        self.m.set_wall_between(11, 15, 1)
        self.m.set_wall_between(3, 7, 1)
        self.m.build_segment_list()
        print(self.m)
        
        fig = plt.figure()
        self.ax1 = fig.add_subplot(1,1,1)
    
    def test_pose_projection_with_plot(self):
        planner = Planner(self.m)
        plan = planner.update_plan([[0, 0], [1, 0], [2, 0], [2, 1]])
        print(planner.curr_plan)
        
        plt.gca().set_aspect('equal', adjustable='box')
        self.m.plot(plt)
        
        # Plot some straight-line poses
        robot_poses = [[0.1, 0.103, np.pi/2 + 0.1],
                       [0.043, 0.22, np.pi/2 - 0.2]]
        for pose in robot_poses:
            self.draw_pose(pose, color='b')
            self.draw_pose(planner.get_reference_pose_from_plan(pose))
            
        # Plot some poses in a macaroni turn:
        robot_poses = [[0.045, 0.367, np.pi/2 - 0.102],
                       [0.036, 0.43, np.pi/2 - 0.3],
                       [0.132, 0.412, np.pi/2 - 1.2],
                       [0.169, 0.440, 0.5],
                       [0.149, 0.459, 0.8]]
        for pose in robot_poses:
            self.draw_pose(pose, color='b')
            self.draw_pose(planner.get_reference_pose_from_plan(pose))
        
        plt.show()
        
    def test_time_parametrized_plan_with_plot(self):
        cells_in_plan = [[0, 0], [1, 0], [2, 0], [2, 1]]
        
        start_pose = [0.1, 0.103, np.pi/2 + 0.1]
        planner = Planner(self.m)
        plan = planner.update_plan(cells_in_plan, start_pose=start_pose)
        print(planner.curr_plan)
        
        plt.gca().set_aspect('equal', adjustable='box')
        self.m.plot(plt)
        
        # Some straight-line poses
        robot_poses = [start_pose,
                       [0.043, 0.22, np.pi/2 - 0.2]]
        pose_times_on_path = []
        for pose in robot_poses:
            pose_times_on_path.append(planner.get_t_on_path(pose))
            
        reference_poses_from_time_parametrization = []
        for pose_time in pose_times_on_path:
            reference_poses_from_time_parametrization.append(
                planner.get_reference_pose_from_plan_by_time(pose_time))
                
        reference_poses_from_projection = []
        for pose in robot_poses:
            reference_poses_from_projection.append(
                planner.get_reference_pose_from_plan(pose))
                
        #############################
        # Compare poses with tolerance in the floating-point arithmetic
        
        # Note: Previously, we were off by p.maze_wall_thickness/2. for an UP cell in the first row, for example
        print(reference_poses_from_time_parametrization)
        print(reference_poses_from_projection)
        #assert reference_poses_from_time_parametrization == reference_poses_from_projection
        
        for i in range(len(reference_poses_from_projection)):
            x_diff = reference_poses_from_projection[i][0] - reference_poses_from_time_parametrization[i][0]
            y_diff = reference_poses_from_projection[i][1] - reference_poses_from_time_parametrization[i][1]
            yaw_diff = reference_poses_from_projection[i][2] - reference_poses_from_time_parametrization[i][2]

            allowed_error = 0.00001
            assert abs(x_diff) <= allowed_error
            assert abs(y_diff) <= allowed_error
            assert abs(yaw_diff) <= allowed_error
            
        ###########################
        
        # Add some poses in a macaroni turn:
        robot_poses += [[0.045, 0.367, np.pi/2 - 0.102],
                        [0.036, 0.43, np.pi/2 - 0.3],
                        [0.132, 0.412, np.pi/2 - 1.2],
                        [0.169, 0.440, 0.5],
                        [0.149, 0.459, 0.8]]
        
        
        
        
        ##########
        plt.show()
        
    def draw_pose(self, pose, color='k', alpha=1.0, size=0.02):
        arrow = mpatches.Arrow(pose[0], pose[1], size*np.cos(pose[2]), size*np.sin(pose[2]),
                               color=color, width=size/2, alpha=alpha)
        self.ax1.add_patch(arrow)
        
        
if __name__ == '__main__':
    planner_test = PlannerTest()
    #planner_test.test_pose_projection_with_plot()
    planner_test.test_time_parametrized_plan_with_plot()