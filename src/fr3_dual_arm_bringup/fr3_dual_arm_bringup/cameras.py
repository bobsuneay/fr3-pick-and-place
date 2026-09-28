"""RealSense device selection and ROS interface shared by launch and tests."""


def camera_parameters(name, serial):
    serial = str(serial).strip().lstrip('_')
    if not serial or not serial.isdecimal():
        raise ValueError(f'{name}: supply the numeric USB serial number; automatic selection is disabled')
    return {
        'camera_name': name, 'serial_no': '_' + serial, 'device_type': 'D405',
        'base_frame_id': 'link', 'publish_tf': True, 'tf_publish_rate': 0.0,
        'enable_color': True, 'enable_depth': True,
        'enable_infra1': False, 'enable_infra2': False,
        'enable_gyro': False, 'enable_accel': False,
        'enable_sync': True, 'align_depth.enable': True,
        'pointcloud.enable': True,
        'depth_module.depth_profile': '1280x720x30',
        'depth_module.color_profile': '1280x720x30',
        'clip_distance': 0.5,
        'color_qos': 'SENSOR_DATA', 'depth_qos': 'SENSOR_DATA',
        'color_info_qos': 'SENSOR_DATA', 'depth_info_qos': 'SENSOR_DATA',
    }


def camera_remappings(name):
    """Use the same public topics as gazebo_ros_camera, without copying messages."""
    return [(f'~/{source}', f'/{name}/{target}') for source, target in (
        ('color/image_raw', 'image_raw'),
        ('color/camera_info', 'camera_info'),
        ('depth/image_rect_raw', 'depth/image_raw'),
        ('depth/camera_info', 'depth/camera_info'),
        ('depth/color/points', 'points'),
        ('aligned_depth_to_color/image_raw', 'aligned_depth/image_raw'),
        ('aligned_depth_to_color/camera_info', 'aligned_depth/camera_info'),
    )]


def selected_cameras(config, serial_overrides=None):
    result = []
    seen = set()
    for side in ('left', 'right'):
        name = side + '_d405'
        entry = config['cameras'][name]
        if not entry.get('enabled', True):
            continue
        serial = (serial_overrides or {}).get(side) or entry.get('serial_no', '')
        params = camera_parameters(name, serial)
        if params['serial_no'] in seen:
            raise ValueError('Left and right D405 must have different serial numbers')
        seen.add(params['serial_no'])
        result.append((name, params))
    return result
