from variables import *

def apply_direction(x, y, dir):
    if dir == left:
        return x - 1, y
    elif dir == right:
        return x + 1, y
    elif dir == up:
        return x, y + 1
    elif dir == down:
        return x, y - 1

class PacBot:
    """
        Allows initializing and updating information about PacBot
    """
    def __init__(self, game_state):
        self.respawn()
        self.game_state = game_state

    def _is_move_legal(self, x, y):
        return self.game_state.grid[x][y] != I and self.game_state.grid[x][y] != n

    def respawn(self):
        self.pos = pacbot_starting_pos
        self.direction = pacbot_starting_dir

    def can_move_direction(self, dir):
        newx, newy = apply_direction(self.pos[0], self.pos[1], dir)
        return self._is_move_legal(newx, newy)

    def update(self):
        newx, newy = apply_direction(self.pos[0], self.pos[1], self.direction)
        if self._is_move_legal(newx, newy):
            self.pos = (newx, newy)
        

#    def update(self, position):
#        if position[0] > self.pos[0]:
#            self.direction = right
#        elif position[0] < self.pos[0]:
#            self.direction = left
#        elif position[1] > self.pos[1]:
#            self.direction = up
#        elif position[1] < self.pos[1]:
#            self.direction = down
#        self.pos = position
