# Offline shared-RPC regression

Run from the repository root with Python 3, g++ and GNU patch:

```bash
python3 tests/shared_rpc/run.py \
  --vendor third_party/frcobot_ros2-v3.0.0_robotV3.9.7/fairino_hardware_v3_9_7 \
  --bash bash --real-sdk-headers
```

The runner copies only selected driver files into a temporary directory. It never
modifies the supplied vendor tree, links its SDK library, or contacts a robot.
It supports the supplied 3.9.7 baseline (with or without the IP patch), the
current gripper patch, and the complete shared-RPC upgrade. Other variants fail
the strict patch preflight instead of being overwritten.

The C++ tests compile the actual patched driver sources with ROS and SDK test
doubles. The fake SDK deliberately rejects a second simultaneous RPC connection
in one process. This reproduces the failure mode under investigation; it is not
proof of the proprietary SDK's internal implementation.

Tests cover activation/release ordering, one RPC per same-IP process, different-IP
rejection, reconnection, failed activation cleanup, real-feedback initialization,
command mapping/deduplication, invalid feedback and SDK error propagation.

`--real-sdk-headers` adds a syntax-only compile against the supplied vendor SDK
headers. ROS types remain test doubles: this does **not** replace a full ROS 2
Humble build, pluginlib runtime load or real-hardware commissioning.

On Windows, `--patch` and `--bash` may point to GNU patch and Git Bash executables.
