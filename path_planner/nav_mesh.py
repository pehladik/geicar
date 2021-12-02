import numpy as np

#x,y avec x→ et y↓
square_center_obstacles=[(5,6),(5,7)]

#board sur 20x20
board=[(0,0),(20,0),(20,20),(0,20)]

graph = { "a" : ["c"],
          "b" : ["c", "e"],
          "c" : ["a", "b", "d", "e"],
          "d" : ["c"],
          "e" : ["c", "b"],
          "f" : []
        } 

class NavMesh2D():
    def __init__(self) -> None:
        pass

