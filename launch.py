from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessStart
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition

def generate_launch_description():
    # Lifecycle map server node
    map_server_node = LifecycleNode(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        namespace = '',
        output='screen',
        parameters=[{
            'yaml_filename': '/home/giuseppina-iannotti/Documents/GITHUB_PROJECTS/SimplePlanner/maps/map.yaml'
        }]
    )

    # Automatic transition: configure → activate
    map_server_configure_and_activate = RegisterEventHandler(
        OnProcessStart(
            target_action=map_server_node,
            on_start=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == map_server_node,
                        transition_id=Transition.TRANSITION_CONFIGURE,
                    )
                ),
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda action: action == map_server_node,
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ]
        )
    )

    return LaunchDescription([
        # Map server and its lifecycle manager
        map_server_node,
        map_server_configure_and_activate,

        # Static transform map → base_link
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='static_tf_pub_map_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'base_link'],
            output='screen'
        ),

        # Simple planner node
        Node(
            package='simple_planner',
            executable='simple_planner_node',
            name='simple_planner_node',
            output='screen'
        ),

        # RViz with config
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=[
                '-d',
                '/home/giuseppina-iannotti/Documents/GITHUB_PROJECTS/SimplePlanner/rviz/simple_planner_config.rviz'
            ],
            output='screen'
        ),
    ])
