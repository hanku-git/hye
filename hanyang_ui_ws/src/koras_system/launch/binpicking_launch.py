import launch
import launch_ros.actions

def generate_launch_description():
    return launch.LaunchDescription([
        launch_ros.actions.Node(
            package='hanyang_eng_koras_system',
            executable='kr_sys',
            name='kr_sys'),
  ])