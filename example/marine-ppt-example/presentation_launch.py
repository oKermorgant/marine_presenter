from simple_launch import SimpleLauncher
from os.path import dirname, join

def generate_launch_description():
    sl = SimpleLauncher()
    
    config_file = join(dirname(__file__), 'config.yaml')
    slider_file = join(dirname(__file__), 'manual.yaml')
    
    sl.declare_arg('remote', default_value=True)
    
    sl.include('marine_presenter', 'bringup_launch.py', 
               launch_arguments=[('config', config_file), 
                                 ('slider', slider_file),
                                 ('remote', sl.arg('remote'))])
               
    # descriptions
    objects = {'bluerov': '/home/olivier/code/ros2/install/marine_presenter/share/marine_presenter/objects/urdf/bluerov.xacro', 'yacht': '/home/olivier/code/ros2/install/marine_presenter/share/marine_presenter/objects/urdf/yacht.xacro', 'turbine': '/home/olivier/code/ros2/install/marine_presenter/share/marine_presenter/objects/urdf/turbine.xacro'}
    fixed_tfs = {'turbine/pole': [10, -10, 10, 1.571, 0, 0]}
    
    for name, filename in objects.items():
        with sl.group(ns=name):
            sl.robot_state_publisher(description_file=filename, xacro_args={'prefix': name})
            
    for link, pose in fixed_tfs.items():
        sl.node('tf2_ros', 'static_transform_publisher', link[:link.find('/')] + '_static', 
                arguments='{} world {}'.format(' '.join(str(c) for c in pose), link))
               
    return sl.launch_description()
