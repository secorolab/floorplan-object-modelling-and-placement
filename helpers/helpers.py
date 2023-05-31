import numpy as np
from .constants import *
from rdflib import RDF

from . import traversal

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

def get_transformation_matrix_wrt_frame(g, root, target):
    filter = [
        GEOM["with-respect-to"],
        GEOM["of"]
        ]
    pred_filter = traversal.filter_by_predicates(filter)
    open_set = traversal.BreadthFirst

    parent_map = {}

    # Traverse the graph
    t_np = traversal.traverse_nodes_with_parent(open_set, g, root, pred_filter)
    for node, parent in ((node, parent) for (node, parent) in t_np if parent):
        parent_map[node] = parent
        if node == target:
            break

    path = []
    curr = target
    while (curr != root):
        path.append(curr)
        curr = parent_map[curr]
    else:
        path.append(root) 

    coordinate_path = []
    for node in path:
        if GEO["Frame"] in g.objects(node, RDF.type):
            continue
        current_frame_coordinates = g.value(predicate=COORD["of-pose"], object=node)
        
        z_value = g.value(current_frame_coordinates, COORD["z"])
        z = 0 if z_value == None else z_value.toPython()
        x = g.value(current_frame_coordinates, COORD["x"]).toPython()
        y = g.value(current_frame_coordinates, COORD["y"]).toPython()
        t = g.value(current_frame_coordinates, FP["theta"]).toPython()

        T = build_transformation_matrix(x, y, z, t)

        coordinate_path.append(T)

    T = np.eye(4)
    for next_T in coordinate_path:
        T = np.dot(next_T, T)
    
    return T