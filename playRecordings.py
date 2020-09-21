import os
import sys
import glob
import time
try:
    sys.path.append(glob.glob('../carla/dist/carla-*%d.%d-%s.egg' % (
        sys.version_info.major,
        sys.version_info.minor,
        'win-amd64' if os.name == 'nt' else 'linux-x86_64'))[0])
except IndexError:
    pass
import carla

try:
    weather = carla.WeatherParameters(
        cloudiness=0.0,
        precipitation=0.0,
        precipitation_deposits=0.0, 
        wind_intensity=0.0, 
        sun_azimuth_angle=70.0, 
        sun_altitude_angle=70.0)                  # Doesn't affect simulation, but does affect visuals

    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    client.reload_world()
    world.get_spectator().set_transform(carla.Transform(carla.Location(x=-135.0, y=-22.50, z=20.), carla.Rotation(yaw=180))) 
    world.set_weather(weather)

    settings = world.get_settings()
    freqSimulation = 100
    if settings.synchronous_mode == False:
        settings.fixed_delta_seconds = 1/freqSimulation
        settings.synchronous_mode = True
        world.apply_settings(settings)
    world.tick()
    # print(client.show_recorder_file_info("/home/jerry/carla/PythonAPI/conflictResolutionCarla/recordings/recording01.log",show_all = 1))
    # client.replay_file("/home/jerry/carla/PythonAPI/conflictResolutionCarla/recordings/recording01.log", 0, 0, 0)
    print(client.show_recorder_file_info("record_20_9_18_1.log",show_all = 0))
    client.replay_file("record_20_9_18_1.log", 0, 0, 0)   
    client.set_replayer_time_factor(1.0)    
    while True:
        world.tick()
        time.sleep(0.01)




finally:
    pass
