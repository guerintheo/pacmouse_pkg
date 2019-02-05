from map import Maze
import params as p

def get_2d_rotation(theta):
    return np.array([[np.cos(theta), -np.sin(theta)],
                     [np.sin(theta), np.cos(theta)]])

class Pose:
    def __init__(self, t=np.zeros(2), r=np.zeros(2,2)):
        assert t.size == r.shape[0] == r.shape[1]
        self.t = t
        self.rotation = r
        self.dim = self.t.size

    def __mul__(self, other):
        # TODO: apply to another pose
        pass

    def inverse(self):
        # TODO: get the inverse
        pass

# TODO:
# * settle on a representation for the robot pose -- rotation, translation (2D?) 
# * choose the origin for the maze coordinate system
# * write a func to get estiamted lidar returns (distributions given a pose and a maze)
# * 

if __name__ = '__main__':
    pass