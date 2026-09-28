"""Preflight checks for a Gazebo launch that owns its own master."""
import os
import socket
from urllib.parse import urlsplit


def require_free_gazebo_master():
    uri = os.environ.get('GAZEBO_MASTER_URI', 'http://localhost:11345')
    address = urlsplit(uri)
    if not address.hostname:
        raise ValueError(f'Invalid GAZEBO_MASTER_URI: {uri}')
    try:
        connection = socket.create_connection((address.hostname, address.port or 11345), timeout=0.3)
    except ConnectionRefusedError:
        return
    with connection:
        pass
    raise RuntimeError(
        f'Gazebo master already listening at {uri}; stop the previous Gazebo simulation '
        'before starting this dual-arm cell')
