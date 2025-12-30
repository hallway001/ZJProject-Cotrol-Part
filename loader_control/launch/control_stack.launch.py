from launch import LaunchDescription
from launch_ros.actions import LifecycleNode, Node
from launch.actions import RegisterEventHandler
from launch.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    # 获取配置文件路径
    config_file = os.path.join(
        get_package_share_directory('loader_control'),
        'config',
        'control.yaml'
    )

    # 状态估计器
    state_estimator = LifecycleNode(
        package='loader_control',
        executable='state_estimator_node',
        name='state_estimator',
        parameters=[config_file],
        namespace='',
    )

    # 轨迹跟踪器
    tracker = LifecycleNode(
        package='loader_control',
        executable='trajectory_tracker_node',
        name='trajectory_tracker',
        parameters=[config_file],
        namespace='',
    )

    # 指令调制器（普通节点，非Lifecycle）
    modulator = Node(
        package='loader_control',
        executable='command_modulator_node',
        name='command_modulator',
        parameters=[config_file],
        namespace='',
    )

    # 控制管理者（主控）
    manager = LifecycleNode(
        package='loader_control',
        executable='control_manager_node',
        name='control_manager',
        parameters=[config_file],
        namespace='',
    )

    # 配置状态估计器为激活状态
    configure_state_estimator = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=state_estimator,
            start_state='unconfigured',
            goal_state='inactive',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=state_estimator,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ],
        )
    )

    # 激活状态估计器
    activate_state_estimator = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=state_estimator,
            start_state='inactive',
            goal_state='active',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=state_estimator,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            ],
        )
    )

    # 配置轨迹跟踪器
    configure_tracker = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=state_estimator,
            start_state='active',
            goal_state='active',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=tracker,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ],
        )
    )

    # 激活轨迹跟踪器
    activate_tracker = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=tracker,
            start_state='inactive',
            goal_state='active',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=tracker,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            ],
        )
    )

    # 配置控制管理器
    configure_manager = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=tracker,
            start_state='active',
            goal_state='active',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=manager,
                    transition_id=Transition.TRANSITION_CONFIGURE,
                )
            ],
        )
    )

    # 激活控制管理器
    activate_manager = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=manager,
            start_state='inactive',
            goal_state='active',
            entities=[
                ChangeState(
                    lifecycle_node_matcher=manager,
                    transition_id=Transition.TRANSITION_ACTIVATE,
                )
            ],
        )
    )

    return LaunchDescription([
        state_estimator,
        tracker,
        modulator,
        manager,
        configure_state_estimator,
        activate_state_estimator,
        configure_tracker,
        activate_tracker,
        configure_manager,
        activate_manager,
    ])

