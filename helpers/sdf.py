import numpy as np
from rdflib import RDF
from .constants import *
from jinja2 import Environment, FileSystemLoader
import os

def get_sdf_geometry(g, polytope):

    if (polytope, RDF.type, FP["CuboidWithSize"]):
        x = g.value(polytope, FP["x-size"])
        y = g.value(polytope, FP["y-size"])
        z = g.value(polytope, FP["z-size"])

        return {
            "type": "box",
            "size": "{x} {y} {z}".format(x=x, y=y, z=z)
        }
    
def get_sdf_intertia(g, inertia):
    sdf_inertia = {}

    sdf_inertia["mass"] = g.value(inertia, RBD["mass"]).toPython()
    sdf_inertia["inertia"] = {}
    sdf_inertia["inertia"]["ixx"] = g.value(inertia, RBD["xx"]).toPython()
    sdf_inertia["inertia"]["iyy"] = g.value(inertia, RBD["yy"]).toPython()
    sdf_inertia["inertia"]["izz"] = g.value(inertia, RBD["zz"]).toPython()

    return sdf_inertia

def get_sdf_pose_from_transformation_matrix(T):

    x = T[0, 3]
    y = T[1, 3]
    z = T[2, 3]

    tx = np.arctan2(T[2,1], T[2, 2])
    ty = np.arctan2(-T[2,0], np.sqrt(T[2,1]**2 + T[2,2]**2))
    tz = np.arctan2(T[1,0], T[0,0])

    return "{x} {y} {z} {tx} {ty} {tz}".format(x=x, y=y, z=z, tx=tx, ty=ty, tz=tz)

def get_sdf_joint_type(g, joint):

    for o in g.objects(joint, RDF.type):
        
        if o == KIN["RevoluteJoint"]:
            return "revolute"
        
def get_sdf_axis_of_rotation(g, joint):
    
    common_axis = g.value(joint, KIN["common-axis"])

    x = 0
    y = 0
    z = 0

    for _, _, line in g.triples((common_axis, GEOM["lines"], None)):
        for _, predicate, _ in g.triples((None, None, line)):
            if predicate == GEO["vector-x"]:
                x = 1
            elif predicate == GEO["vector-y"]:
                y = 1
            elif predicate == GEO["vector-z"]:
                z = 1

    return "{x} {y} {z}".format(x=x, y=y, z=z)

def write_object_model_sdf(data, output_folder):

    file_loader = FileSystemLoader('templates')
    env = Environment(loader=file_loader)
    
    name_without_id = data["name"][3:]
    full_path = os.path.join(output_folder, name_without_id)

    if not os.path.exists(full_path):
        print("does not exists")
        os.makedirs(full_path)

    template = env.get_template('model.sdf.jinja')
    output = template.render(data=data, trim_blocks=True, lstrip_blocks=True)

    with open(os.path.join(full_path, "model.sdf"), "w") as f:
        f.write(output)
        print("{name} MODEL FILE: {path}".format(name=name_without_id, path=os.path.join(full_path, "model.sdf")))

    template = env.get_template('model.config.jinja')
    output = template.render(data=data, trim_blocks=True, lstrip_blocks=True)

    with open(os.path.join(full_path, "model.config".format(name=name_without_id)), "w") as f:
        f.write(output)
        print("{name} CONFIG FILE: {path}".format(name=name_without_id, path=os.path.join(full_path, "{name}.config".format(name=name_without_id))))
