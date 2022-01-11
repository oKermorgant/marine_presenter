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
    objects = <objects>
    fixed_tfs = <fixed_tfs>
    
    for name, filename in objects.items():
        with sl.group(ns=name):
            sl.robot_state_publisher(description_file=filename, xacro_args={'prefix': name})
            
    for link, pose in fixed_tfs.items():
        sl.node('tf2_ros', 'static_transform_publisher', link[:link.find('/')] + '_static', 
                arguments=[str(c) for c in pose] + ['world', link])        
               
    return sl.launch_description()
