import carla
import math
import random
import time
import numpy
import weakref

client = carla.Client('localhost', 2000)
world = client.get_world()

ego_bp = world.get_blueprint_library().find('vehicle.tesla.model3')
ego_bp.set_attribute('role_name','ego')
print('\nEgo role_name is set ')
print(ego_bp)

ego_color = random.choice(ego_bp.get_attribute('color').recommended_values)
ego_bp.set_attribute('color',ego_color)
print('\nEgo color is set')
print(ego_color)

spawn_points = world.get_map().get_spawn_points()
number_of_spawn_points = len(spawn_points)
#print(spawn_points)
#print(number_of_spawn_points)

rad_bp = None
rad_bp = world.get_blueprint_library().find('sensor.other.radar')
vehicle_blueprints = world.get_blueprint_library().filter('*vehicle*')
ego_vehicle = world.spawn_actor(ego_bp, random.choice(spawn_points))
rad_bp.set_attribute('horizontal_fov', str(35))
rad_bp.set_attribute('vertical_fov', str(20))
rad_bp.set_attribute('range', str(20))
rad_location = carla.Location(x=2.0, z=1.0)
rad_rotation = carla.Rotation(roll=3.0, pitch=1.0, yaw=2.0)
rad_transform = carla.Transform(rad_location,rad_rotation)
rad_ego = world.spawn_actor(rad_bp,rad_transform,attach_to=ego_vehicle, attachment_type=carla.AttachmentType.Rigid)

print(rad_bp)
print(rad_location)
print(rad_rotation)
print(rad_transform)
#print(ego_vehicle)
#print(rad_ego)

class World(object):
    def __init__(self, ):
        pass

class RadarSensor(object):
    def __init__(self, parent_actor):
        self.sensor = None
        self._parent = parent_actor
        bound_x = 0.5 + self._parent.bounding_box.extent.x
        bound_y = 0.5 + self._parent.bounding_box.extent.y
        bound_z = 0.5 + self._parent.bounding_box.extent.z

        self.velocity_range = 7.5 # m/s
        world = self._parent.get_world()
        self.debug = world.debug
        bp = world.get_blueprint_library().find('sensor.other.radar')
        bp.set_attribute('horizontal_fov', str(35))
        bp.set_attribute('vertical_fov', str(20))
        self.sensor = world.spawn_actor(
            bp,
            carla.Transform(
                carla.Location(x=bound_x + 0.05, z=bound_z+0.05),
                carla.Rotation(pitch=5)),
            attach_to=self._parent)
        # We need a weak reference to self to avoid circular reference.
        weak_self = weakref.ref(self)
        self.sensor.listen(
            lambda radar_data: RadarSensor.rad_callback(weak_self, radar_data))

    def rad_callback(weak_self, radar_data):
        self = weak_self()
        if not self:
            return
        # To get a numpy [[vel, altitude, azimuth, depth],...[,,,]]:
        points = numpy.frombuffer(radar_data.raw_data, dtype=numpy.dtype('f4'))
        points = numpy.reshape(points, (len(radar_data), 4))
        #print("Points = ", points)

        velocity_range = 7.5 # m/s
        current_rot = radar_data.transform.rotation
        print(current_rot)
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