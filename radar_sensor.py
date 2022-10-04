from __future__ import print_function

import glob
import os
import sys

try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass

import carla 
import math 
import random 
import time 
import numpy as np
import cv2
import open3d as o3d
from matplotlib import cm

#Set up the client using the CARLA client object
client = carla.Client('localhost', 2000)
#client.load_world('Town04')
client.set_timeout(10.0) #seconds
world = client.get_world()
bp_lib = world.get_blueprint_library() 
spawn_points = world.get_map().get_spawn_points()

# --------------
# Add a new radar sensor to my ego
# --------------

# --------------
# Spawn ego vehicle
# --------------
ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
ego_bp.set_attribute('role_name','ego')
print('\nEgo role_name is set')
ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
ego_bp.set_attribute('color',ego_color)
print('\nEgo color is set')

spawn_points = world.get_map().get_spawn_points()
number_of_spawn_points = len(spawn_points)

if 0 < number_of_spawn_points:
    random.shuffle(spawn_points)
    ego_transform = spawn_points[0]
    ego_vehicle = world.spawn_actor(ego_bp,ego_transform)
    print('\nEgo is spawned')
else: 
    logging.warning('Could not found any spawn points')
 
rad_cam = None
rad_bp = world.get_blueprint_library().find('sensor.other.radar')
rad_bp.set_attribute('horizontal_fov', str(35))
rad_bp.set_attribute('vertical_fov', str(20))
rad_bp.set_attribute('range', str(20))
rad_location = carla.Location(x=2.0, z=1.0)
rad_rotation = carla.Rotation(pitch=5)
rad_transform = carla.Transform(rad_location,rad_rotation)
rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)
def rad_callback(radar_data):
    velocity_range = 7.5 # m/s
    current_rot = radar_data.transform.rotation
    for detect in radar_data:
        azi = math.degrees(detect.azimuth)
        alt = math.degrees(detect.altitude)
        # The 0.25 adjusts a bit the distance so the dots can
        # be properly seen
        fw_vec = carla.Vector3D(x=detect.depth - 0.25)
        carla.Transform(
            carla.Location(),
            carla.Rotation(
                pitch=current_rot.pitch + alt,
                yaw=current_rot.yaw + azi,
                roll=current_rot.roll)).transform(fw_vec)

        def clamp(min_v, max_v, value):
            return max(min_v, min(value, max_v))

        norm_velocity = detect.velocity / velocity_range # range [-1, 1]
        r = int(clamp(0.0, 1.0, 1.0 - norm_velocity) * 255.0)
        g = int(clamp(0.0, 1.0, 1.0 - abs(norm_velocity)) * 255.0)
        b = int(abs(clamp(- 1.0, 0.0, - 1.0 - norm_velocity)) * 255.0)
        world.debug.draw_point(
            radar_data.transform.location + fw_vec,
            size=0.075,
            life_time=0.06,
            persistent_lines=False,
            color=carla.Color(r, g, b))
rad_ego.listen(lambda radar_data: rad_callback(radar_data))