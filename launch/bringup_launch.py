from simple_launch import SimpleLauncher

def generate_launch_description():
    sl = SimpleLauncher()
    
    sl.declare_arg('config', default_value = '', description='path to the config file')    
    sl.declare_arg('slider', default_value= '', description='path to the slider file')
    sl.declare_arg('remote', default_value=True)

    sl.node('slider_publisher', 'slider_publisher', name='slides', arguments=[sl.arg('slider')])
    
    sl.node('marine_presenter', 'presenter.py', arguments=[sl.arg('config')], output='screen')
    
    with sl.group(if_arg='remote'):
        sl.node('marine_presenter', 'remote.py', output='screen')
    
    return sl.launch_description()
