import carla
import math
import random
import time

#Set up the client using the CARLA client object
client = carla.Client('localhost', 2000)
client.set_timeout(10.0) #seconds
world = client.get_world()

client.load_world('Town03')

blueprint_library = world.get_blueprint_library()
spawn_points = world.get_map().get_spawn_points()

radar_sensor_bp = blueprint_library.find('sensor.other.radar')
vehicle_bp = blueprint_library.find('vehicle.lincoln.mkz_2020')
vehicle = world.try_spawn_actor(vehicle_bp, random.choice(spawn_points))

vehicles = blueprint_library.filter('vehicle.*')
bikes = [x for x in vehicles if int(x.get_attribute('number_of_wheels')) == 2]
for bike in bikes:
    bike.set_attribute('color', '255,0,0')
    
for attr in vehicle_bp:
    if attr.is_modifiable:
        vehicle_bp.set_attribute(attr.id, random.choice(attr.recommended_values))

spectator = world.get_spectator()
transform = carla.Transform(vehicle.get_transform().transform(carla.Location(x=230, y=40)), vehicle.get_transform().rotation)
actor = world.spawn_actor(vehicle_bp, transform)
spectator.set_transform(transform)

for bp in blueprint_library.filter('radar'):
    print(bp.id)





