#!/home/ruijiliu/anaconda3/envs/rapids-23.08/bin/python3

import xml.etree.ElementTree as ET
import random
def create_sdf_world(num_rows, bushes_per_row, bush_spacing, row_spacing):
    sdf = ET.Element('sdf', version='1.7')
    world = ET.SubElement(sdf, 'world', name='default')

    # Light element
    light = ET.SubElement(world, 'light', name='sun', type='directional')
    ET.SubElement(light, 'cast_shadows').text = '1'
    ET.SubElement(light, 'pose').text = '0 0 10 0 -0 0'
    ET.SubElement(light, 'diffuse').text = '0.8 0.8 0.8 1'
    ET.SubElement(light, 'specular').text = '0.2 0.2 0.2 1'
    attenuation = ET.SubElement(light, 'attenuation')
    ET.SubElement(attenuation, 'range').text = '1000'
    ET.SubElement(attenuation, 'constant').text = '0.9'
    ET.SubElement(attenuation, 'linear').text = '0.01'
    ET.SubElement(attenuation, 'quadratic').text = '0.001'
    ET.SubElement(light, 'direction').text = '-0.5 0.1 -0.9'
    spot = ET.SubElement(light, 'spot')
    ET.SubElement(spot, 'inner_angle').text = '0'
    ET.SubElement(spot, 'outer_angle').text = '0'
    ET.SubElement(spot, 'falloff').text = '0'

    # Ground plane model
    ground_plane = ET.SubElement(world, 'model', name='ground_plane')
    ET.SubElement(ground_plane, 'static').text = '1'
    link = ET.SubElement(ground_plane, 'link', name='link')

    # Collision element for ground plane
    collision = ET.SubElement(link, 'collision', name='collision')
    geometry = ET.SubElement(collision, 'geometry')
    plane = ET.SubElement(geometry, 'plane')
    ET.SubElement(plane, 'normal').text = '0 0 1'
    ET.SubElement(plane, 'size').text = '100 100'
    # ... Add other elements to collision as needed

    # Visual element for ground plane
    visual = ET.SubElement(link, 'visual', name='visual')
    ET.SubElement(visual, 'cast_shadows').text = '0'
    geometry_vis = ET.SubElement(visual, 'geometry')
    plane_vis = ET.SubElement(geometry_vis, 'plane')
    ET.SubElement(plane_vis, 'normal').text = '0 0 1'
    ET.SubElement(plane_vis, 'size').text = '100 100'
    # ... Add other elements to visual as needed
    surface = ET.SubElement(collision, 'surface')
    contact = ET.SubElement(surface, 'contact')
    ET.SubElement(contact, 'collide_bitmask').text = '65535'
    ET.SubElement(contact, 'ode')

    # Friction sub-element
    friction = ET.SubElement(surface, 'friction')
    ode_friction = ET.SubElement(friction, 'ode')
    ET.SubElement(ode_friction, 'mu').text = '100'
    ET.SubElement(ode_friction, 'mu2').text = '50'

    # Torsional sub-element
    torsional = ET.SubElement(friction, 'torsional')
    ET.SubElement(torsional, 'ode')

    # Bounce sub-element
    ET.SubElement(surface, 'bounce')

    # Max contacts element
    ET.SubElement(collision, 'max_contacts').text = '10'

    material = ET.SubElement(visual, 'material')
    script = ET.SubElement(material, 'script')
    ET.SubElement(script, 'uri').text = 'file://media/materials/scripts/gazebo.material'
    ET.SubElement(script, 'name').text = 'Gazebo/Grey'

    # Set self_collide, enable_wind, and kinematic for the link
    ET.SubElement(link, 'self_collide').text = '0'
    ET.SubElement(link, 'enable_wind').text = '0'
    ET.SubElement(link, 'kinematic').text = '0'
    
    # Add gravity, magnetic_field, etc. here
    # Gravity element
    ET.SubElement(world, 'gravity').text = '0 0 -9.8'

    # Magnetic field element
    ET.SubElement(world, 'magnetic_field').text = '6e-06 2.3e-05 -4.2e-05'

    ET.SubElement(world, 'atmosphere', type='adiabatic')

    # Physics element with sub-elements
    physics = ET.SubElement(world, 'physics', type='ode')
    ET.SubElement(physics, 'max_step_size').text = '0.001'
    ET.SubElement(physics, 'real_time_factor').text = '1'
    ET.SubElement(physics, 'real_time_update_rate').text = '1000'

    # Scene element with sub-elements
    scene = ET.SubElement(world, 'scene')
    ET.SubElement(scene, 'ambient').text = '0.4 0.4 0.4 1'
    ET.SubElement(scene, 'background').text = '0.7 0.7 0.7 1'
    ET.SubElement(scene, 'shadows').text = '1'

    # Wind element
    ET.SubElement(world, 'wind')

    # Spherical coordinates element with sub-elements
    spherical_coordinates = ET.SubElement(world, 'spherical_coordinates')
    ET.SubElement(spherical_coordinates, 'surface_model').text = 'EARTH_WGS84'
    ET.SubElement(spherical_coordinates, 'latitude_deg').text = '0'
    ET.SubElement(spherical_coordinates, 'longitude_deg').text = '0'
    ET.SubElement(spherical_coordinates, 'elevation').text = '0'
    ET.SubElement(spherical_coordinates, 'heading_deg').text = '0'
    # Function to create a bush model element
    def create_bush_model(bush_id, x, y, z, scale):
        name = f'bush_1_nc_{bush_id}'
        model = ET.Element('model', name=name)
        ET.SubElement(model, 'static').text = '1'

        link = ET.SubElement(model, 'link', name='link')

        visual = ET.SubElement(link, 'visual', name='visual')
        geometry = ET.SubElement(visual, 'geometry')
        mesh = ET.SubElement(geometry, 'mesh')
        # ET.SubElement(mesh, 'uri').text = f'model://bush_1_nc/meshes/Bush_31.obj'
        ET.SubElement(mesh, 'uri').text = f'model://corn_stalk/meshes/corn_1.obj'
        ET.SubElement(mesh, 'scale').text = f'{0.002} {0.002} {0.001}'

        material = ET.SubElement(visual, 'material')
        script = ET.SubElement(material, 'script')
        uri_material = ET.SubElement(script, 'uri')
        uri_material.text = 'file://media/materials/scripts/gazebo.material'
        name = ET.SubElement(script, 'name')
        name.text = 'Gazebo/Grass'

        ET.SubElement(link, 'self_collide').text = '0'
        ET.SubElement(link, 'enable_wind').text = '0'
        ET.SubElement(link, 'kinematic').text = '0'

        ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 -0 0'

        return model
    # def create_bush_model(bush_id, x, y, z, scale):
    #     name = f'bush_1_{bush_id}'
    #     model = ET.Element('model', name=name)
    #     ET.SubElement(model, 'static').text = '1'
        
    #     link = ET.SubElement(model, 'link', name='link')
        
    #     collision = ET.SubElement(link, 'collision', name='collision')
    #     geometry = ET.SubElement(collision, 'geometry')
    #     mesh = ET.SubElement(geometry, 'mesh')
    #     ET.SubElement(mesh, 'uri').text = f'model://bush_1/meshes/Bush_31.obj'
    #     ET.SubElement(mesh, 'scale').text = f'{scale} {scale} {scale}'

    #     surface = ET.SubElement(collision, 'surface')
    #     friction = ET.SubElement(surface, 'friction')
    #     ode_friction = ET.SubElement(friction, 'ode')
    #     ET.SubElement(ode_friction, 'mu').text = '100'
    #     ET.SubElement(ode_friction, 'mu2').text = '50'

    #     torsional = ET.SubElement(friction, 'torsional')
    #     ET.SubElement(torsional, 'ode')

    #     contact = ET.SubElement(surface, 'contact')
    #     ET.SubElement(contact, 'ode')

    #     ET.SubElement(surface, 'bounce')

    #     ET.SubElement(collision, 'max_contacts').text = '10'

    #     visual = ET.SubElement(link, 'visual', name='visual')
    #     geometry_vis = ET.SubElement(visual, 'geometry')
    #     mesh_vis = ET.SubElement(geometry_vis, 'mesh')
    #     ET.SubElement(mesh_vis, 'uri').text = f'model://bush_1/meshes/Bush_31.obj'
    #     ET.SubElement(mesh_vis, 'scale').text = f'{scale} {scale} {scale}'

    #     ET.SubElement(link, 'self_collide').text = '0'
    #     ET.SubElement(link, 'enable_wind').text = '0'
    #     ET.SubElement(link, 'kinematic').text = '0'

    #     ET.SubElement(model, 'pose').text = f'{x} {y} {z} 0 -0 0'

    #     return model
    i = 0
    # Generate bush models in rows
    for row in range(1, num_rows):
        y = row * row_spacing 
        # if (y/row_spacing) % 5 == 0 and y != 0:
        #     pass
        # else:
        z = 0.171977
        for bush in range(bushes_per_row):
            error = random.uniform(-0.1, 0.1)
            # print(error)
            y = row * row_spacing #+ error
            x = bush * bush_spacing
            # if (x/bush_spacing) % 5 == 0 and x!=0:
            #     pass
            # else:
            world.append(create_bush_model(i, x, y, z, 0.0003))
            i += 1

    state = ET.SubElement(world, 'state', world_name='default')

    ET.SubElement(state, 'sim_time').text = '44 154000000'
    ET.SubElement(state, 'real_time').text = '44 571842198'
    ET.SubElement(state, 'wall_time').text = '1699303547 287877280'
    ET.SubElement(state, 'iterations').text = '44154'

    # State for each model
    j = 0
    # Generate bush models in rows
    for row in range(1, num_rows):
        y = row * row_spacing
        # if (y/row_spacing) % 5 == 0 and y != 0:
        #     pass
        # else:
        z = 0.171977
        for bush in range(bushes_per_row):
            error = random.uniform(-0.1, 0.1)
            # print(error)
            y = row * row_spacing #+ error
            x = bush * bush_spacing
            # if (x/bush_spacing) % 5 == 0 and x !=0:
            #     pass
            # else:
            model_state = ET.SubElement(state, 'model', name=f'bush_1_nc_{j}')
            ET.SubElement(model_state, 'pose').text = f'{x} {y} {z} 0 -0 0'
            ET.SubElement(model_state, 'scale').text = '1 1 1'
            link_state = ET.SubElement(model_state, 'link', name='link')
            ET.SubElement(link_state, 'pose').text = f'{x} {y} {z} 0 -0 0'
            ET.SubElement(link_state, 'velocity').text = '0 0 0 0 -0 0'
            ET.SubElement(link_state, 'acceleration').text = '0 0 0 0 -0 0'
            ET.SubElement(link_state, 'wrench').text = '0 0 0 0 -0 0'
            j += 1

    # State for ground plane
    ground_plane_state = ET.SubElement(state, 'model', name='ground_plane')
    ET.SubElement(ground_plane_state, 'pose').text = '0 0 0 0 -0 0'
    ET.SubElement(ground_plane_state, 'scale').text = '1 1 1'
    link_gp_state = ET.SubElement(ground_plane_state, 'link', name='link')
    ET.SubElement(link_gp_state, 'pose').text = '0 0 0 0 -0 0'
    ET.SubElement(link_gp_state, 'velocity').text = '0 0 0 0 -0 0'
    ET.SubElement(link_gp_state, 'acceleration').text = '0 0 0 0 -0 0'
    ET.SubElement(link_gp_state, 'wrench').text = '0 0 0 0 -0 0'

    # State for the sun light
    sun_state = ET.SubElement(state, 'light', name='sun')
    ET.SubElement(sun_state, 'pose').text = '0 0 10 0 -0 0'

    # GUI element
    gui = ET.SubElement(world, 'gui', fullscreen='0')
    camera = ET.SubElement(gui, 'camera', name='user_camera')
    ET.SubElement(camera, 'pose').text = '4.26149 0.036101 4.2799 -0 0.711644 3.02819'
    ET.SubElement(camera, 'view_controller').text = 'orbit'
    ET.SubElement(camera, 'projection_type').text = 'perspective'
    # Convert to a string and write to a file
    tree = ET.ElementTree(sdf)
    ET.indent(tree, space="\t", level=0)
    tree.write('/home/ruijiliu/catkin_ws/src/iowa_navigation/worlds/test_corn.world', encoding='utf-8', xml_declaration=False)
    print("finished")
# Example usage: create a world with 2 rows of bushes, 5 bushes per row, bushes spaced by 2 meters, rows spaced by 1 meter
if __name__ == "__main__":
    # Example usage of the function
    create_sdf_world(num_rows=15, bushes_per_row=40, bush_spacing=0.5, row_spacing=0.762)