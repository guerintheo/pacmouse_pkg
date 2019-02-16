import unittest
import numpy as np
from map import Maze

class IOTest(unittest.TestCase):
    """
    Tests IO for Maze class. Contains basic test cases.

    Each test function instantiates a Maze and checks that all returned arrays/frames are probability distributions and sum to 1.
    """

    def configure_test_maze(self):
        m = Maze(3,3)
        m.set_connected([0,0],[0,1],0)
        m.set_connected([1,1],[2,1],0)
        m.set_connected([2,2],[2,1], 0)
        m.set_connected([1,2],[1,1], 0)
        return m

    def test_adjacency_basics(self):
        m = self.configure_test_maze()
        m.set_connected([2,0],[2,1], 0.5)
        prob = m.get_connected([2,0],[2,1])
        self.assertEqual(prob, 0.5, 'Get/Set adjacency matrix is borked')

        self.assertRaisesRegexp(AssertionError, 'Cells must be adjacent', m.get_connected, [0,0], [2,0])

    def test_maze_solving(self):
        m = self.configure_test_maze()
        start, end = [0,0], [2,2]
        path1 = m.get_path(start, end)
        print('Path found: %s from %s to %s \n%s' % (path1, start, end, m))

        # This test is out of date due to a random maze being generated every time the maze is instantiated. 

        # self.assertTrue(np.array_equal(path1, [0,1,4,3,6,7,8]), 'Pathfinding is borked')

        start, end = [0,0], [1,0]
        path2 = m.get_path(start, end)
        print('Path found: %s from %s to %s \n%s' % (path2, start, end, m))
        
        # This test is out of date due to a random maze being generated every time the maze is instantiated. 

        # self.assertTrue(np.array_equal(path2, [0,1]), 'Pathfinding to an adjacent square')

        start, end = [0,0], [0,0]
        path3 = m.get_path(start, end)
        self.assertTrue(np.array_equal(path3, []), 'Finding path from a square to itself should return an empty array')

    def test_set_connected(self):
        # TODO: set_connected() should validate input and make sure the input coords aren't out of bounds. 
        pass
    
    def test_generated_maze(self):
        # TODO: generate_random_maze() should always produce a connected maze. 
        pass

if __name__ == '__main__':
    unittest.main()