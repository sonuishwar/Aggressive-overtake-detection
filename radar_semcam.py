import carla
import pandas as pd
import numpy as np
import random
import time
import cv2

# Connect to the CARLA server
client = carla.Client('localhost', 2000)
client.set_timeout(10.0)

# Get the world object
world = client.get_world()

# Get blueprint for the ego vehicle
ego_vehicle_bp = random.choice(world.get_blueprint_library().filter('vehicle'))

# Get spawn points for ego vehicle and following vehicle
ego_spawn_point = carla.Transform(carla.Location(x=44.070000, y=-5.840000, z=4.150000), carla.Rotation(yaw=0))
following_spawn_point = carla.Transform(carla.Location(x=ego_spawn_point.location.x - 10, y=ego_spawn_point.location.y, z=ego_spawn_point.location.z))

# Spawn the ego vehicle
ego_vehicle = world.spawn_actor(ego_vehicle_bp, ego_spawn_point)

# Spawn the following vehicle
following_vehicle = world.spawn_actor(ego_vehicle_bp, following_spawn_point)

# # Let both vehicles drive around.
# ego_vehicle.set_autopilot(True)
# following_vehicle.set_autopilot(True)


# Calculate the offset to attach the sensor behind the ego vehicle
ego_bbox = ego_vehicle.bounding_box

offset_x = -ego_bbox.extent.x  # Place it behind the ego vehicle
offset_y = 0  # Centered horizontally
offset_z = ego_bbox.extent.z + 1  # Slightly above the ground level

# Define the location for attaching the semantic segmentation camera behind the ego vehicle
rel_location = carla.Location(offset_x, offset_y, offset_z)


# Load blueprint for the radar sensor
radar_bp = world.get_blueprint_library().find('sensor.other.radar')
radar_bp.set_attribute('horizontal_fov', '90')
radar_bp.set_attribute('vertical_fov', '5')
radar_bp.set_attribute('range', '100')
radar_bp.set_attribute('sensor_tick', str(0.1))  # Adjust the tick rate as needed

# Attach the radar sensor to the ego vehicle
radar_location = carla.Location(2, 0, 1)
radar_rotation = carla.Rotation(yaw=180, pitch=0, roll=0)
radar_transform = carla.Transform(rel_location, radar_rotation)
radar_sensor = world.spawn_actor(radar_bp, radar_transform, attach_to=ego_vehicle)

# Load blueprint for the semantic segmentation camera
sem_cam_bp = world.get_blueprint_library().find('sensor.camera.semantic_segmentation')
sem_cam_bp.set_attribute('image_size_x', '800')
sem_cam_bp.set_attribute('image_size_y', '600')
sem_cam_bp.set_attribute("fov", str(90))
sem_cam_bp.set_attribute('sensor_tick', str(0.1))  # Adjust the tick rate as needed

# Attach the semantic segmentation camera to the ego vehicle
sem_location = carla.Location(2, 0, 1)
sem_rotation = carla.Rotation(yaw=180, pitch=0, roll=0)
sem_cam_transform = carla.Transform(rel_location, sem_rotation)
# sem_cam_transform = carla.Transform(carla.Location(x=2, y=0, z=1))
sem_cam = world.spawn_actor(sem_cam_bp, sem_cam_transform, attach_to=ego_vehicle,
                            attachment_type=carla.AttachmentType.Rigid)

# Define lists to store radar and semantic segmentation data
radar_data_list = []
sem_seg_data_list = []

# Subscribe to radar sensor data
def process_radar_data(data):
    radar_data = np.frombuffer(data.raw_data, dtype=np.dtype('f4'))
    radar_data = np.reshape(radar_data, (len(data), 4))  # Each radar data point has 4 values: [velocity, altitude, azimuth, depth]
    for detection in radar_data:
        velocity = detection[0]
        altitude = detection[1]
        azimuth = detection[2]
        depth = detection[3]
        if is_following_vehicle(velocity, azimuth):
            radar_data_list.append({
                'time': time.time(),
                'velocity': velocity,
                'altitude': altitude,
                'azimuth': azimuth,
                'depth': depth
            })

def is_following_vehicle(velocity, azimuth):
    return velocity < 0 and abs(azimuth) < np.pi / 2  # Adjust conditions as needed

# Subscribe to semantic segmentation camera data
def process_sem_seg_data(image):
    # Save the image captured by the semantic camera
    image.save_to_disk('./Semantic_images/%.6d.jpg' % image.frame, carla.ColorConverter.CityScapesPalette)
    
    # Convert semantic segmentation image to numpy array
    array = np.frombuffer(image.raw_data, dtype=np.dtype("uint8"))
    array = np.reshape(array, (image.height, image.width, 4))
    
    # Define the range of blue channel values for cars
    min_blue = 1
    max_blue = 255
    
    # Extract data for cars
    car_mask = np.logical_and(array[..., 2] >= min_blue, array[..., 2] <= max_blue)
    
    # Check if any car pixels are found
    if np.any(car_mask):
        # Find contours of detected cars
        contours, _ = cv2.findContours(car_mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        
        # Process contours and extract data
        for contour in contours:
            x, y, w, h = cv2.boundingRect(contour)
            center_x = x + w / 2
            center_y = y + h / 2
            # print("Detected car at: (x:", center_x, ", y:", center_y, "), width:", w, ", height:", h)
            sem_seg_data_list.append({
                'time': time.time(),
                'center_x': center_x,
                'center_y': center_y,
                'width': w,
                'height': h
            })
    else:
        print("No cars detected in the image.")



# Subscribe to radar and semantic segmentation camera data
radar_sensor.listen(process_radar_data)
sem_cam.listen(process_sem_seg_data)
# sem_cam.listen(lambda image: image.save_to_disk('./Semantic_images/%.6d.jpg' % image.frame,
                                                        # carla.ColorConverter.CityScapesPalette))

# Run simulation for some time
time.sleep(3)  # Adjust as needed

# Convert the radar and semantic segmentation data to pandas DataFrames
radar_df = pd.DataFrame(radar_data_list)
sem_seg_df = pd.DataFrame(sem_seg_data_list)

radar_df.to_csv('radar4.csv',index=False)
sem_seg_df.to_csv('sem_cam4.csv',index=False)

# Check if both DataFrames have data
if not radar_df.empty and not sem_seg_df.empty:
    # Merge the DataFrames on the 'time' column
    combined_df = pd.merge(radar_df, sem_seg_df, on='time', how='inner')

    # Save the combined data to a CSV file
    combined_df.to_csv('combined_data.csv', index=False)
else:
    print("Error: One or both DataFrames are empty. Cannot merge.")

# Destroy actors
ego_vehicle.destroy()
following_vehicle.destroy()
radar_sensor.destroy()
sem_cam.destroy()
