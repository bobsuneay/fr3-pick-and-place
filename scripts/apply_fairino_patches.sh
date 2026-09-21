#!/usr/bin/env bash
# Preflight on a copy, then back up and install only the affected source files.
set -euo pipefail
repo_root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"
vendor_root="$(cd -- "${1:-$repo_root/third_party/frcobot_ros2-v3.0.0_robotV3.9.7}" && pwd)"
package=fairino_hardware_v3_9_7
command -v patch >/dev/null
stage="$(mktemp -d "${TMPDIR:-/tmp}/fr3-sdk-stage.XXXXXXXX")"
trap 'rm -rf -- "$stage"' EXIT
files=(CMakeLists.txt fairino_hardware.xml
  include/fairino_hardware/fairino_hardware_interface.hpp
  src/fairino_hardware_interface.cpp)
optional=(include/fairino_hardware/fairino_gripper_hardware_interface.hpp
  include/fairino_hardware/shared_robot_connection.hpp
  src/fairino_gripper_hardware_interface.cpp)
mkdir -p "$stage/$package/include/fairino_hardware" "$stage/$package/src"
for file in "${files[@]}"; do
  cp -- "$vendor_root/$package/$file" "$stage/$package/$file"
done
for file in "${optional[@]}"; do
  if [[ -f "$vendor_root/$package/$file" ]]; then
    files+=("$file")
    cp -- "$vendor_root/$package/$file" "$stage/$package/$file"
  fi
done
apply() { (cd -- "$stage" && patch --force --no-backup-if-mismatch --fuzz=0 -p1 "$@" < "$repo_root/third_party/$patch_name"); }
patch_name=fairino_shared_rpc.patch
if ! apply --dry-run -R >/dev/null 2>&1; then
  for patch_name in fairino_dual_arm_ip.patch fairino_gripper_interface.patch fairino_shared_rpc.patch; do
    if apply --dry-run -R >/dev/null 2>&1; then continue; fi
    apply --forward --dry-run
    apply --forward
  done
fi
# The shared patch changes the gripper patch's context. Verify in reverse order.
cp -R "$stage/$package" "$stage/verify"
for patch_name in fairino_shared_rpc.patch fairino_gripper_interface.patch fairino_dual_arm_ip.patch; do
  apply -R
done
changed=false
for file in "${files[@]}" "${optional[@]}"; do
  if ! cmp -s "$stage/verify/$file" "$vendor_root/$package/$file"; then changed=true; fi
done
if [[ "$changed" == false ]]; then echo 'Already installed.'; exit 0; fi
backup="$(mktemp "${TMPDIR:-/tmp}/fr3-sdk-backup.XXXXXXXX.tar.gz")"
tar -czf "$backup" -C "$vendor_root/$package" "${files[@]}"
echo "Original source backup: $backup"
for file in "${files[@]}" "${optional[@]}"; do
  cp -- "$stage/verify/$file" "$vendor_root/$package/$file"
done
echo 'PASS: SDK gripper and shared connection installed. Rebuild fairino_hardware_v3_9_7.'
