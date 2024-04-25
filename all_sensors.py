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
import numpy as np
import random
import time
import argparse
import logging
SetAutopilot = carla.command.SetAutopilot
# print_over_same_line = carla.util
SpawnActor = carla.command.SpawnActor
# TCPConnectionError = carla.tcp
FutureActor = carla.command.FutureActor
def get_information(actors): # only works with vehicle, not walker
    location = actors.get_location()
    acceleration = actors.get_acceleration()
    speed = actors.get_velocity()
    vehicle_transform = actors.get_transform()
    return location,acceleration,speed,vehicle_transform

def distance_ego_vehicles(ego_location,vehicles_location):
    a = np.square(ego_location.x - vehicles_location.x)
    b = np.square(ego_location.y - vehicles_location.y)
    c = np.square(ego_location.z - vehicles_location.z)
    distance = np.sqrt(a + b + c)
    return distance

def radar_event(raw_data,role_of_actors):
    # To get a numpy [[vel, azimuth, altitude, depth],...[,,,]]:
    points = np.frombuffer(raw_data.raw_data, dtype=np.dtype('f4'))
    points = np.reshape(points, (len(raw_data), 4))
    velocity = points[:, 0]
    print(f'Velocity of {role_of_actors} = {velocity}')
    # return velocity

def imu_retrieve_information(imu_information, name, id=None):
    location_list = []
    timstamp_list = []
    dict_information = {}
    location = imu_information.transform.location
    time = imu_information.timestamp
    acceleration = imu_information.accelerometer
    orientation = imu_information.compass
    print(f'Timestamp of {name} is {time}')
    print(f'Location of {name} is {location}')
    print(f'Acceleration of {name} is {acceleration}')
    print(f'Orientation of {name} is {orientation}')

    timstamp_list.append(time)
    print('len of timestamp list', len(timstamp_list))
    if id is not None:
        dict_information[id].append([location, acceleration, orientation])
        if id not in dict_information.keys():
            dict_information.update({id:[location, acceleration, orientation]})
        print(dict_information)



def _main():
    argparser = argparse.ArgumentParser()
    argparser.add_argument(
        '-s', '--save-images-to-disk',
        action='store_true',
        dest='save_images_to_disk',
        help='save images into disk'
    )
    argparser.add_argument(
        '-i', '--show-actor-info',
        action='store_true',
        dest='show_info_actors',
        help='show the actor information'
    )
    argparser.add_argument(
        '-so', '--save-other-side-image',
        action='store_true',
        dest='save_other_side_images',
        help='save images on other side of the vehicle'
    )
    argparser.add_argument(
        '--tm-port',
        metavar='P',
        default=8000,
        type=int,
        help='port to communicate with TM (default: 8000)')
    argparser.add_argument(
        '-n', '--number-of-vehicles',
        metavar='N',
        default=10,
        type=int,
        help='number of vehicles (default: 10)')
    argparser.add_argument(
        '--safe',
        action='store_true',
        help='avoid spawning vehicles prone to accidents')
    args = argparser.parse_args()
    # log_level = logging.DEBUG if args.debug else logging.INFO
    # logging.basicConfig(format='%(levelname)s: %(message)s', level=log_level)

    while True:
        main(args)


def main(args):
    actor_list = []

    try:
        client = carla.Client('localhost', 2000)
        client.set_timeout(15.0)


        world = client.get_world()
        blueprint_library = world.get_blueprint_library()

        bp = random.choice(blueprint_library.filter('vehicle'))

        if bp.has_attribute('color'):
            color = random.choice(bp.get_attribute('color').recommended_values)
            bp.set_attribute('color', color)

        transform = random.choice(world.get_map().get_spawn_points())

        vehicle = world.spawn_actor(bp, transform)

        actor_list.append(vehicle)
        print('created %s' % vehicle.type_id)

        # Let's put the vehicle to drive around.
        vehicle.set_autopilot(True)


        # Add depth images
        camera_bp = None
        camera_bp = blueprint_library.find('sensor.camera.depth')
        camera_bp.set_attribute("image_size_x", str(800))
        camera_bp.set_attribute("image_size_y", str(600))
        camera_bp.set_attribute("fov", str(90))
        camera_transform = carla.Transform(carla.Location(x=1.5, z=2.4))
        camera = world.spawn_actor(camera_bp, camera_transform, attach_to=vehicle)
        actor_list.append(camera)
        print('created %s' % camera.type_id)

        # Now we register the function that will be called each time the sensor
        # receives an image. In this example we are saving the image to disk
        # converting the pixels to gray-scale.
        cc = carla.ColorConverter.LogarithmicDepth

# ------------------------Add rgb camera-------------------------------------
        camera_rgb = None
        camera_rgb = blueprint_library.find('sensor.camera.rgb')
        camera_rgb.set_attribute("image_size_x", str(800))
        camera_rgb.set_attribute("image_size_y", str(600))
        camera_rgb.set_attribute("fov", str(90))
        camera_rgb_transform = carla.Transform(carla.Location(x=1.7, z=2.6))
        camera_rgb_spawn = world.spawn_actor(camera_rgb,camera_rgb_transform,attach_to=vehicle)
        actor_list.append(camera_rgb_spawn)
        print('create %s' % camera_rgb_spawn.type_id)

        # camera rgb left
        camera_rgb_left = None
        camera_rgb_bp_left = blueprint_library.find('sensor.camera.rgb')
        camera_rgb_bp_left.set_attribute("image_size_x", str(800))
        camera_rgb_bp_left.set_attribute("image_size_y", str(600))
        camera_rgb_bp_left.set_attribute("fov", str(90))
        camera_rgb_transform_left = carla.Transform(carla.Location(x=1.7, z=2.6),
                                                    carla.Rotation(0, 270, 0))
        camera_rgb_left = world.spawn_actor(camera_rgb_bp_left, camera_rgb_transform_left, attach_to=vehicle)
        actor_list.append(camera_rgb_left)
        print('create rgb left %s' % camera_rgb_left.type_id)

        # camera rgb right
        camera_rgb_right = None
        camera_rgb_bp_right = blueprint_library.find('sensor.camera.rgb')
        camera_rgb_bp_right.set_attribute("image_size_x", str(800))
        camera_rgb_bp_right.set_attribute("image_size_y", str(600))
        camera_rgb_bp_right.set_attribute("fov", str(90))
        camera_rgb_transform_right= carla.Transform(carla.Location(x=1.7, z=2.6),
                                                    carla.Rotation(0, 90, 0))
        camera_rgb_right = world.spawn_actor(camera_rgb_bp_right, camera_rgb_transform_right, attach_to=vehicle)
        actor_list.append(camera_rgb_right)
        print('create rgb left %s' % camera_rgb_right.type_id)

        # camera rgb back
        camera_rgb_back = None
        camera_rgb_bp_back = blueprint_library.find('sensor.camera.rgb')
        camera_rgb_bp_back.set_attribute("image_size_x", str(800))
        camera_rgb_bp_back.set_attribute("image_size_y", str(600))
        camera_rgb_bp_back.set_attribute("fov", str(90))
        camera_rgb_transform_back = carla.Transform(carla.Location(x=1.7, y=3.0, z=2.6),
                                                     carla.Rotation(0, 180, 0))
        camera_rgb_back = world.spawn_actor(camera_rgb_bp_back, camera_rgb_transform_back, attach_to=vehicle)
        actor_list.append(camera_rgb_back)
        print('create rgb left %s' % camera_rgb_back.type_id)


#----------------------------------------------------------------------------------------------------------------

        # Add semantic camera
        sem_cam = None
        sem_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
        sem_bp.set_attribute("image_size_x", str(800))
        sem_bp.set_attribute("image_size_y", str(600))
        sem_bp.set_attribute("fov", str(90))
        sem_location = carla.Location(2, 0, 1)
        sem_rotation = carla.Rotation(0, 0, 0)
        sem_transform = carla.Transform(sem_location, sem_rotation)
        sem_cam = world.spawn_actor(sem_bp, sem_transform, attach_to=vehicle,
                                    attachment_type=carla.AttachmentType.Rigid)
        actor_list.append(sem_cam)
        # This time, a color converter is applied to the image, to get the semantic segmentation view



        # Add Lidar data
        lidar_cam = None
        lidar_bp = world.get_blueprint_library().find('sensor.lidar.ray_cast')
        lidar_bp.set_attribute('channels', str(32))
        lidar_bp.set_attribute('points_per_second', str(90000))
        lidar_bp.set_attribute('rotation_frequency', str(40))
        lidar_bp.set_attribute('range', str(20))
        lidar_location = carla.Location(2, 0, 1)
        lidar_rotation = carla.Rotation(0, 0, 0)
        lidar_transform = carla.Transform(lidar_location, lidar_rotation)
        lidar_sen = world.spawn_actor(lidar_bp, lidar_transform, attach_to=vehicle)



        # Get IMU sensor
        blueprints_imu = world.get_blueprint_library().find('sensor.other.imu')
        blueprints_imu.set_attribute('noise_accel_stddev_x',str(1.0))
        blueprints_imu.set_attribute('noise_accel_stddev_y', str(1.0))
        blueprints_imu.set_attribute('noise_accel_stddev_z', str(1.0))
        blueprints_imu.set_attribute('noise_gyro_stddev_x', str(1.0))
        blueprints_imu.set_attribute('noise_gyro_stddev_y', str(1.0))
        blueprints_imu.set_attribute('noise_gyro_stddev_z', str(1.0))
        imu_location = carla.Location(2, 0, 1)
        imu_rotation = carla.Rotation(0, 0, 0)
        imu_transform = carla.Transform(imu_location, imu_rotation)
        imu_sensor = world.spawn_actor(blueprints_imu, imu_transform, attach_to=vehicle)
        actor_list.append(imu_sensor)


        # Get Rada sensor
        blueprints_radar = world.get_blueprint_library().find('sensor.other.radar')
        blueprints_radar.set_attribute('sensor_tick',str(0.5)) # Capture information in each 0.5 seconds
        radar_location = carla.Location(2, 0, 1)
        radar_rotation = carla.Rotation(0, 0, 0)
        radar_transform = carla.Transform(radar_location, radar_rotation)
        radar_sensor = world.spawn_actor(blueprints_radar, radar_transform, attach_to=vehicle)
        actor_list.append(radar_sensor)


        #---------------SAVE IMAGES TO DISK-----------------------------
        # Save the forward images
        if args.save_images_to_disk:
            camera.listen(lambda image: image.save_to_disk('/media/data/luu/RGB_images/forward_img/%06d.png' % image.frame, cc))

            sem_cam.listen(lambda image: image.save_to_disk('/media/data/luu/Semantic_images/%.6d.jpg' % image.frame,
                                                        carla.ColorConverter.CityScapesPalette))
            camera_rgb_spawn.listen(
                lambda image: image.save_to_disk('/media/data/luu/RGB_images/%06d.png' % image.frame))

            lidar_sen.listen(
                lambda point_cloud: point_cloud.save_to_disk(
                    '/media/data/luu/Lidar_images/%.6d.ply' % point_cloud.frame))

        # Save the sides and back images
        if args.save_other_side_images:

            camera_rgb_left.listen(lambda image: image.save_to_disk('/media/data/luu/RGB_images/side_L_img/%06d.png'% image.frame))
            camera_rgb_right.listen(lambda image: image.save_to_disk('/media/data/luu/RGB_images/side_R_img/%06d.png'% image.frame))
            camera_rgb_back.listen(lambda image: image.save_to_disk('/media/data/luu/RGB_images/side_back_img/%06d.png'% image.frame))

        #--------------------------------------------------------------------------------------------------------


        location = vehicle.get_location()
        location.x += 40
        vehicle.set_location(location)
        print('moved vehicle to %s' % location)


#Region

        spawn_points = world.get_map().get_spawn_points()

        blueprints = world.get_blueprint_library().filter('vehicle.*')

        # Get agent vehicle IMU sensor
        agent_blueprints_imu = world.get_blueprint_library().find('sensor.other.imu')
        agent_blueprints_imu.set_attribute('noise_accel_stddev_x', str(1.0))
        agent_blueprints_imu.set_attribute('noise_accel_stddev_y', str(1.0))
        agent_blueprints_imu.set_attribute('noise_accel_stddev_z', str(1.0))
        agent_blueprints_imu.set_attribute('noise_gyro_stddev_x', str(1.0))
        agent_blueprints_imu.set_attribute('noise_gyro_stddev_y', str(1.0))
        agent_blueprints_imu.set_attribute('noise_gyro_stddev_z', str(1.0))

        agent_imu_transform = carla.Transform(imu_location, imu_rotation)

        # Get agent vehicle RADAR sensor
        agent_blueprints_radar = world.get_blueprint_library().find('sensor.other.radar')
        agent_blueprints_radar.set_attribute('sensor_tick', str(0.5))  # Capture information in each 0.5 seconds
        agent_radar_location = carla.Location(2, 0, 1)
        agent_radar_rotation = carla.Rotation(0, 0, 0)
        agent_radar_transform = carla.Transform(agent_radar_location, agent_radar_rotation)


        if args.safe:
            blueprints = [x for x in blueprints if int(x.get_attribute('number_of_wheels')) == 4]
            blueprints = [x for x in blueprints if not x.id.endswith('isetta')]
            blueprints = [x for x in blueprints if not x.id.endswith('carlacola')]

        blueprints = sorted(blueprints, key=lambda bp_: bp_.id)
        traffic_manager = client.get_trafficmanager(args.tm_port)
        traffic_manager.set_global_distance_to_leading_vehicle(1.0)

        vehicle_batch = []
        vehicle_id = []
        agent_sensor_list = []
        for n, transform in enumerate(spawn_points):
            if n >= args.number_of_vehicles:
                break
            blueprint = random.choice(blueprints)
            if blueprint.has_attribute('color'):
                color = random.choice(blueprint.get_attribute('color').recommended_values)
                blueprint.set_attribute('color', color)
            if blueprint.has_attribute('driver_id'):
                driver_id = random.choice(blueprint.get_attribute('driver_id').recommended_values)
                blueprint.set_attribute('driver_id', driver_id)
                vehicle_id.append(driver_id)

            transform.location += carla.Location(x=8)

            npc = world.try_spawn_actor(blueprint, transform)
            if npc is not None:
                npc.set_autopilot(True)
                # Spawn imu sensor
                agent_imu_sensor = world.spawn_actor(agent_blueprints_imu, agent_imu_transform, attach_to=npc)

                # # Spawn radar sensor
                # agent_radar_sensor = world.spawn_actor(agent_blueprints_radar, agent_radar_transform,attach_to=npc)

                # actor_list.append(agent_imu_sensor)
                actor_list.append(npc)
                agent_sensor_list.append(agent_imu_sensor)
                # agent_sensor_list.append(agent_radar_sensor)


                # if args.show_info_actors:
                    # Retrieve the agent vehicles information
                print('--------------------------------------------------------')
                print('----------------Agent vehicles information--------------')
                print('--------------------------------------------------------')
                print('created npc %s' % npc.type_id)
                agent_imu_sensor.listen(
                    lambda information: imu_retrieve_information(information,'Agent ' + str(npc.type_id)
                                                                 )
                )

                # agent_radar_sensor.listen(lambda raw_data: radar_event(raw_data, 'Agent vehicle'))
                print('--------------------------------------------------------')



#Endregion

    #Region
        # # Spawn 1 npc to test
        # transform_npc = random.choice(world.get_map().get_spawn_points())
        # # transform.location.x += 8.0
        # bp_npc = random.choice(blueprint_library.filter('vehicle'))
        # npc = world.try_spawn_actor(bp_npc, transform_npc)
        # # if npc is not None:
        # actor_list.append(npc)
        # npc.set_autopilot(True)
        #
        # agent_vehicle_location, agent_vehicle_acceleration, \
        # agent_vehicle_speed, agent_vehicle_transform = get_information(npc)
        #
        # agent_vehicle_info.append([agent_vehicle_location, agent_vehicle_acceleration,
        #                            agent_vehicle_speed, agent_vehicle_transform])
        # # Results: We can see the information of 1 npc
    #Endregion

        # # Vehicle's information
        if args.show_info_actors:
            print('--------------------------------------------------------')
            print('----------------Ego vehicle information--------------')
            print('--------------------------------------------------------')

            imu_sensor.listen(
                lambda information: imu_retrieve_information(information,'Ego ' + str(vehicle.type_id))
            )
            # radar_sensor.listen(lambda raw_data: radar_event(raw_data, 'Ego vehicle'))
            print('----------------------------------------------------')



# SPAWN WALKERS
# region
        # Add some walkers
        blueprintsWalkers = world.get_blueprint_library().filter('walker.pedestrian.*')
        percentagePedestriansRunning = 0.0  # how many pedestrians will run
        percentagePedestriansCrossing = 0.0  # how many pedestrians will walk through the road
        # 1. take all the random locations to spawn
        spawn_points = []
        walkers_list = []
        walker_id = []
        for i in range(0,20):
            spawn_point = carla.Transform()
            loc = world.get_random_location_from_navigation()
            if (loc != None):
                spawn_point.location = loc
                spawn_points.append(spawn_point)
        # 2. we spawn the walker object
        walker_batch = []
        walker_speed = []
        for spawn_point in spawn_points:
            walker_bp = random.choice(blueprintsWalkers)
            # set as not invincible
            if walker_bp.has_attribute('is_invincible'):
                walker_bp.set_attribute('is_invincible', 'false')
            # set the max speed
            if walker_bp.has_attribute('speed'):
                if (random.random() > percentagePedestriansRunning):
                    # walking
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[1])
                else:
                    # running
                    walker_speed.append(walker_bp.get_attribute('speed').recommended_values[2])
            else:
                print("Walker has no speed")
                walker_speed.append(0.0)
            walker_batch.append(SpawnActor(walker_bp, spawn_point))
        results = client.apply_batch_sync(walker_batch, True)
        walker_speed2 = []
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list.append({"id": results[i].actor_id})
                walker_speed2.append(walker_speed[i])
        walker_speed = walker_speed2
        # 3. we spawn the walker controller
        batch = []
        walker_controller_bp = world.get_blueprint_library().find('controller.ai.walker')
        for i in range(len(walkers_list)):
            batch.append(SpawnActor(walker_controller_bp, carla.Transform(), walkers_list[i]["id"]))
        results = client.apply_batch_sync(batch, True) # check command.Response to check its return value and usage
        for i in range(len(results)):
            if results[i].error:
                logging.error(results[i].error)
            else:
                walkers_list[i]["con"] = results[i].actor_id

        # 4. we put altogether the walkers and controllers id to get the objects from their id
        for i in range(len(walkers_list)):
            walker_id.append(walkers_list[i]["con"])
            walker_id.append(walkers_list[i]["id"])

        all_actors = world.get_actors(walker_id)

        # 5. initialize each controller and set target to walk to (list is [controler, actor, controller, actor ...])
        # set how many pedestrians can cross the road
        world.set_pedestrians_cross_factor(percentagePedestriansCrossing)
        for i in range(0, len(walker_id), 2):
            # start walker
            all_actors[i].start()
            # set walk to random point
            all_actors[i].go_to_location(world.get_random_location_from_navigation())
            # max speed
            all_actors[i].set_max_speed(float(walker_speed[int(i / 2)]))
#endregion

        while True:
            if args.show_info_actors:


                # print('------------Walkers vehicle information----------------')

                # if npc is not None:
                #     location_npc = npc.get_location()
                #     acceleration_npc = npc.get_acceleration()
                #     speed_npc = npc.get_velocity()
                #     vehicle_transform_npc = npc.get_transform()
                #     print('agent vehicle ID %s' % npc.type_id)
                #     print('Location of the ego vehicle = ', location_npc)
                #     print('acceleration of the ego vehicle = ', acceleration_npc)
                #     print('speed of the ego vehicle = ', speed_npc)
                #     print('transform of the ego vehicle = ', vehicle_transform_npc)

                # for i in vehicle_batch:
                #     print('what is in vehicle batch', i.transform)


                # for agent_vehicle in agent_vehicle_list:
                #     print('speed',agent_vehicle.get_velocity())


                print('--------------------------------------------')
                print(' ')
                print('----------walkers information---------------')
                # for i in walker_speed2:
                #     print('this is what is in walker_speed2',i)
                for i in range(len(walkers_list)):
                    # print('controller is ',walkers_list[i]["con"])
                    print('ID of walker is ',walkers_list[i]["id"])
                    print('Walker speed = ',walker_speed[i])
                    print('walker location is ',walker_batch[i].transform)
                    # print('Location and Rotation', batch[i].transform)

                # for i in batch:
                #     print('this is what is in batch list',i.transform)
                print('---------------------------------------------')

                # Vehicle's information
                world.tick()
                print('----------Ego vehicle information-----------------')
                print('Velocity of the ego vehicle = ', vehicle.get_velocity())
                # print('Acceleration of ego vehicle = ', vehicle.get_acceleration())
                # print('Location of ego vehicle = ', vehicle.get_transform())
                print('----------------------------------------------------')

                # Test npc information
                # if npc is not None:
                #     print('npc ID', npc.type_id)
                #     print('npc velocity ', npc.get_velocity())

        time.sleep(30)
        # while True:
        #     world_snapshot = world.wait_for_tick()
    finally:

        print('destroying depth camera')
        camera.destroy()

        # Destroy rgb camera
        print('Destroying rgb cameras')
        camera_rgb_spawn.destroy()
        camera_rgb_back.destroy()
        camera_rgb_right.destroy()
        camera_rgb_left.destroy()

        print('Destroying semantic camera')
        sem_cam.destroy()

        print('Destroying sensors on the ego vehicle')
        lidar_sen.destroy()
        imu_sensor.destroy()
        radar_sensor.destroy()

        print('Destroying sensors on the agent vehicles')
        for agent_sensor in agent_sensor_list:
            agent_sensor.destroy()
        # client.apply_batch([carla.command.DestroyActor(x) for x in agent_imu_sensor_list])
        client.apply_batch([carla.command.DestroyActor(x) for x in actor_list])


        print('destroying walkers')
        # stop walker controllers (list is [controller, actor, controller, actor ...])
        for i in range(0, len(walker_id), 2):
            all_actors[i].stop()

        print('\ndestroying %d walkers' % len(walkers_list))
        client.apply_batch([carla.command.DestroyActor(x) for x in walker_id])

        print('done.')



if __name__ == '__main__':
    try:
        _main()
    except KeyboardInterrupt:
        print('cancelled by user')