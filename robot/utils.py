def calc_move(xold, yold, heading):
    heading_dist = {
        0: (0,1),
        1: (-1, 1),
        2: (-1, 0),
        3: (-1, -1),
        4: (0, -1),
        5: (1, -1),
        6: (1, 0),
        7: (1, 1)
    }
    xnew = xold + heading_dist[heading][0]
    ynew = yold + heading_dist[heading][1]
    return  xnew, ynew


def calc_turn(heading_old, turncount):
    return ((heading_old + turncount) % 8)

def calc_uturn(heading_old):
    return ((heading_old + 4) % 8)