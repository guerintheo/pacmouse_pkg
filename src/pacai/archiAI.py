import sys
from collections import namedtuple
from functools import reduce

from gameState import *

SEARCH_DEPTH = 20
MINIMAX_SD = 8 # change?
DIRS = [ right, left, up, down ]


def print_state(state):
    width = len(state.grid)
    height = len(state.grid[0])
    chars = []
    for x in range(width):
        chars.append([])
        for y in range(height):
            chars[-1].append('X')

    for x in range(width):
        chars.append([])
        for y in range(height):
            cell = state.grid[x][y]
            if state.pacbot.pos == (x,y):
                chars[x][y] = 'P'
            elif state.red.pos[0] == (x,y):
                chars[x][y] = 'G'
            elif state.blue.pos[0] == (x,y):
                chars[x][y] = 'G'
            elif state.orange.pos[0] == (x,y):
                chars[x][y] = 'G'
            elif state.pink.pos[0] == (x,y):
                chars[x][y] = 'G'
            elif cell == I:
                chars[x][y] = '|'
            elif cell == o:
                chars[x][y] = 'o'
            elif cell == O:
                chars[x][y] = 'O'
            else:
                chars[x][y] = ' '
    for y in range(height - 1, -1, -1):
        for x in range(width):
            sys.stdout.write(chars[x][y])
        sys.stdout.write('\n')

GameStateNode = namedtuple('GameStateNode', ('state', 'moves' ))

def opposite_dir(dir):
    return dir ^ 1

def advance_state(state):
    state.pacbot.update()
    for i in range(ticks_per_update):
        state.next_step()

def child_nodes(node):
    children = [ None ] * 4
    for i in range(4):
        dir = DIRS[i]
        if dir != opposite_dir(node.state.pacbot.direction) and node.state.pacbot.can_move_direction(dir):
            c = copy.deepcopy(node)
            c.state.pacbot.direction = dir
            c.moves.append(dir)
            advance_state(c.state)
            children[i] = c
    return children

def tree_search(node, depth):
    if depth == 0 or not node.state.play:
        return node
    children = child_nodes(node)
    results = [tree_search(c, depth - 1) for c in children if c]
    best = reduce(lambda a, b : a if a.state.score > b.state.score else b, results)
    return best

def ab_child_nodes(node, maximizing):
    if maximizing:
        return child_nodes(node)
    children = [ None ] * 4 * 4
    ghosts = [node.state.red, node.state.pink, node.state.orange, node.state.blue]
    for i in range(4):
        ghost = ghosts[i]
        for j in range(4):
            dir = DIRS[j]
            if dir != opposite_dir(ghost.direction) and ghost.can_move_direction(dir):
                c = copy.deepcopy(node)
                ghost.force_next_move(dir)
                advance_state(c.state)
                children[4*i+j] = c
    return children

def archibald_bronson_search(node, depth, archibald, bronson, maximizing):
    if depth == 0 or not node.state.play:
        return node
    children = ab_child_nodes(node, maximizing)
    if maximizing:
        bestScore = -100000
        bestState = None
        for c in children:
            if not c:
                continue
            res = archibald_bronson_search(c, depth - 1, archibald, bronson, False)
            if res.state.score > bestScore:
                bestScore = res.state.score
                bestState = res
            if res.state.score > archibald:
                archibald = res.state.score
            if archibald >= bronson:
                break
        return bestState
    else:
        worstScore = 100000
        worstState = None
        for c in children:
            if not c:
                continue
            res = archibald_bronson_search(c, depth - 1, archibald, bronson, True)
            if res.state.score < worstScore:
                worstScore = res.state.score
                worstState = res
            if res.state.score < bronson:
                bronson = res.state.score
            if archibald >= bronson:
                break
        return worstState

def ai_next_move(origin):
    return tree_search(GameStateNode(origin, []), SEARCH_DEPTH).moves

def ai_next_move_ab(origin):
    best_path = archibald_bronson_search(GameStateNode(origin, []), MINIMAX_SD, -100000, 100000, True)
    return best_path.moves
 

def main():
    state = GameState()
    state.unpause()
    while state.play:
        best_path = ai_next_move_ab(state) if state.start_counter < 40 else ai_next_move(state)
        state.pacbot.direction = best_path[0]

        state.pacbot.update()
        for i in range(ticks_per_update):
            state.next_step()

        print_state(state)
        strs = ['r', 'l', 'u', 'd']
        for d in best_path:
            sys.stdout.write(strs[d] + ' ')
        sys.stdout.write('\n')

        strs = [ '', 's', 'c', 'f']
        print(strs[state.state])
        input()

if __name__ == "__main__":
    main()
