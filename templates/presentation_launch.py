from simple_launch import SimpleLauncher
from os.path import dirname, join


def generate_launch_description():
    sl = SimpleLauncher()
    
    config_file = join(dirname(__file__), 'config.yaml')
    slider_file = join(dirname(__file__), 'manual.yaml')
    
    sl.declare_arg('remote', default_value=True)
    sl.declare_arg('coral', default_value=True)
    
    sl.include('marine_presenter', 'bringup_launch.py',
               launch_arguments={'config': config_file,
                                 'slider': slider_file,
                                 'remote': sl.arg('remote')})
    
    # environment
    with sl.group(if_arg='coral'):
        sl.node('coral', 'coral_gui')
    sl.service('/coral/spawn', request= {'world_model': sl.find('coral','islands.urdf')})
               
    # descriptions
    objects = <objects>
    fixed_tfs = <fixed_tfs>
    
    for name, filename in objects.items():
        with sl.group(ns=name):
            sl.robot_state_publisher(description_file=filename, xacro_args={'prefix': name})
    sl.service('/coral/spawn')
            
    for link, pose in fixed_tfs.items():
        sl.node('tf2_ros', 'static_transform_publisher', link[:link.find('/')] + '_static',
                arguments=[str(c) for c in pose] + ['world', link])
               
    return sl.launch_description()
