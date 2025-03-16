from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Indítjuk a turtlesim node-ot
        Node(
            package='turtlesim',  
            executable='turtlesim_node',  
            name='turtlesim',  
            output='screen'  
        ),
        
        # Indítjuk a project_node node-ot
        Node(
            package='kir_i4v_random',  #
            executable='draw_random',  
            name='draw_random1',  
            output='screen'  
        )
    ])
