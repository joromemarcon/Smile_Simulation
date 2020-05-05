'''
    Simplified: Waypoint traversal
'''
import math
import random
import time

_startingPosition = (0,0)
_currentPosition = _startingPosition
_nextPosition = _currentPosition

_checkpoints = {
                (0,0):[(0,1),(4,8),(5,9)],
                (0,1):[(3,7),(2,6)],
                (4,8):[(3,7)],
                (5,9):[(2,6)],
                (3,7):[(0,0),(2,6),(5,9)],
                (2,6):[(4,8)],
                }


while(1):
    _currentPosition = _nextPosition
    if(_currentPosition == _nextPosition):
        testPoint = _checkpoints[_currentPosition]
        index = random.randint(1, len(testPoint))
        _nextPosition = testPoint[index-1]

    time.sleep(2)

    print(_currentPosition)

#
# testPoint = _checkpoints[_startingPosition]
# index = random.randint(1,len(testPoint))
# print("index: ", index-1)
# testNextPos = testPoint[index-1]
#
# print(testPoint)
# print(len(testPoint))
# print(testNextPos)