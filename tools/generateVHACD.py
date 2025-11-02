import xml.etree.ElementTree as ET
import pybullet as p
import os
import trimesh


def stl_to_obj(input_stl, folder_stl, folder_obj) :
    # Charger le fichier STL
    mesh = trimesh.load_mesh(os.path.join(folder_stl, input_stl+".stl"))
    
    # Exporter en format OBJ
    output_obj = os.path.join(folder_obj, input_stl+".obj")
    mesh.export(output_obj)


def convert_vhacd2(input_obj, folder_obj, depth=10) :
    p.connect(p.DIRECT)

    output_obj = os.path.join(folder_obj, input_obj+"_vhacd2.obj")
    name_log = os.path.join(folder_obj, "log.txt")

    p.vhacd(os.path.join(folder_obj, input_obj+".obj"), output_obj, name_log, depth=depth)

def update_urdf(input_stl_list, folder_obj, new_urdf_path) :
    # Create new urdf
    root = urdf_tree.getroot()

    parent_new_urdf = os.path.dirname(new_urdf_path)
    relative_path = os.path.relpath(folder_obj, start=parent_new_urdf)
    relative_path = relative_path.replace("\\", "/")

    # Parcours tous les liens du fichier
    for link in root.findall("link"):
        collision = link.find("collision")
        if collision is not None:
            mesh = collision.find(".//mesh")
            if mesh is not None:
                old_file = os.path.basename(mesh.get("filename").replace(".stl", ""))
                if old_file in input_stl_list:
                    new_file = relative_path + "/" + old_file + "_vhacd2.obj"
                    mesh.set("filename", new_file)
                    print(f"  → {old_file} replace by {new_file}")

    # Sauvegarder le fichier modifié
    urdf_tree.write(new_urdf_path, encoding="utf-8", xml_declaration=True)



##############################################################################
#############################       Param      ###############################
##############################################################################



folder_stl = "Robot_mesh_urdf_V3/meshes"
folder_obj = "Robot_mesh_urdf_V3/meshes/collision_form"
new_urdf_path = "Robot_mesh_urdf_V3/RobotSpider_With_Col.urdf"
urdf_tree = ET.parse("Robot_mesh_urdf_V3/RobotSpider.urdf")

input_stl_list = ["Rotation_Horrizontale"]
deth = 10

if not os.path.exists(folder_obj):
    os.mkdir(folder_obj)

for input_stl in input_stl_list :
        stl_to_obj(input_stl, folder_stl, folder_obj)
        convert_vhacd2(input_stl, folder_obj, deth)
        os.remove(os.path.join(folder_obj, input_stl+".obj"))

update_urdf(input_stl_list, folder_obj, new_urdf_path)

##############################################################################




