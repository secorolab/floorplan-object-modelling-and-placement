import numpy as np
from .constants import *

def loader(directory):
    import os
    def load(file):
        with open(os.path.join(directory, file)) as f:
            return f.read()
        return ""
    return load

def prefixed(g, node):
    return node.n3(g.namespace_manager)

def build_transformation_matrix(x, y, z, theta):
    
    c = np.cos 
    s = np.sin

    t = np.array([[x], [y], [z], [1]])
    R = np.array([
        [c(theta), -s(theta), 0],
        [s(theta), c(theta), 0],
        [0, 0, 1],
        [0, 0, 0]]
    )

    return np.hstack((R, t))

def get_transformation_matrix_wrt_frame(g, frame, target_frame):

    current_frame = frame 
    coordinate_path = []

    while (current_frame != target_frame):

        current_frame_pose = g.value(subject=None, predicate=GEOM["of"], object=current_frame)
        current_frame_coordinates = g.value(subject=None, predicate=COORD["of-pose"], object=current_frame_pose)
        
        x = g.value(current_frame_coordinates, COORD["x"]).toPython()
        y = g.value(current_frame_coordinates, COORD["y"]).toPython()
        z = g.value(current_frame_coordinates, COORD["z"]).toPython()
        t = g.value(current_frame_coordinates, FP["theta"]).toPython()

        T = build_transformation_matrix(x, y, z, t)

        coordinate_path.append(T)

        current_frame = g.value(current_frame_coordinates , COORD["as-seen-by"])

    T = np.eye(4)
    for next_T in coordinate_path:
        T = np.dot(next_T, T)
    
    return T