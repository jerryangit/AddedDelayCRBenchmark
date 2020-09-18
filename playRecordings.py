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
    client = carla.Client('localhost', 2000)
    client.set_timeout(2.0)
    world = client.get_world()
    settings = world.get_settings()
    freqSimulation = 100
    if settings.synchronous_mode == False:
        settings.fixed_delta_seconds = 1/freqSimulation
        settings.synchronous_mode = True
        world.apply_settings(settings)
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
