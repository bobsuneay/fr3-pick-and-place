#!/usr/bin/env python3
"""Offline driver regression: compile real driver sources against ROS/SDK doubles.

Only a temporary copy is patched. Never links the vendor SDK or contacts robots.
The optional second compile checks SDK signatures against the supplied real headers.
"""
import argparse
from pathlib import Path
import shutil
import subprocess
import tempfile


def run(argv, **kwargs):
    print('+', ' '.join(map(str, argv)), flush=True)
    return subprocess.run(list(map(str, argv)), check=True, **kwargs)


def patch_command(executable, root, patch_file, *, reverse=False, dry=True):
    command = [executable, '--batch', '--fuzz=0', '-p1', '-i', patch_file]
    if reverse:
        command.append('-R')
    else:
        command.append('--forward')
    if dry:
        command.append('--dry-run')
    return subprocess.run(list(map(str, command)), cwd=root, capture_output=True)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--vendor', type=Path, required=True,
                        help='Path to the fairino_hardware_v3_9_7 package')
    parser.add_argument('--cxx', default='g++')
    parser.add_argument('--patch', default='patch')
    parser.add_argument('--real-sdk-headers', action='store_true')
    parser.add_argument('--bash', help='Also test the installer script and repeat invocation')
    args = parser.parse_args()
    vendor = args.vendor.resolve()
    tests = Path(__file__).resolve().parent
    patches = tests.parents[1] / 'third_party'
    selected = ['CMakeLists.txt', 'fairino_hardware.xml',
                'src/fairino_hardware_interface.cpp',
                'include/fairino_hardware/fairino_hardware_interface.hpp']
    optional = ['src/fairino_gripper_hardware_interface.cpp',
                'include/fairino_hardware/fairino_gripper_hardware_interface.hpp',
                'include/fairino_hardware/shared_robot_connection.hpp']
    with tempfile.TemporaryDirectory(prefix='fr3_shared_rpc_') as temporary:
        root = Path(temporary)
        package = root / 'fairino_hardware_v3_9_7'
        for name in selected + [n for n in optional if (vendor / n).exists()]:
            destination = package / name
            destination.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(vendor / name, destination)

        shared = patches / 'fairino_shared_rpc.patch'
        if args.bash:
            installer = tests.parents[1] / 'scripts/apply_fairino_patches.sh'
            shell = [args.bash, '-c', 'export PATH="/usr/bin:/bin:$PATH"; exec bash "$@"',
                     'bash', installer.as_posix(), root.as_posix()]
            run(shell)
            before = {p.relative_to(package): p.read_bytes() for p in package.rglob('*') if p.is_file()}
            run(shell)
            after = {p.relative_to(package): p.read_bytes() for p in package.rglob('*') if p.is_file()}
            assert before == after, 'Repeat installer invocation changed the source'
            print('PASS: installer applies all patches and is idempotent')
        already_shared = patch_command(args.patch, root, shared, reverse=True)
        if already_shared.returncode != 0:
            for name in ['fairino_dual_arm_ip.patch', 'fairino_gripper_interface.patch',
                         'fairino_shared_rpc.patch']:
                patch_file = patches / name
                check = patch_command(args.patch, root, patch_file)
                if check.returncode == 0:
                    applied = patch_command(args.patch, root, patch_file, dry=False)
                    if applied.returncode:
                        raise RuntimeError(applied.stdout.decode(errors='replace'))
                    print('APPLIED:', name)
                elif patch_command(args.patch, root, patch_file, reverse=True).returncode == 0:
                    print('ALREADY APPLIED:', name)
                else:
                    raise RuntimeError('Patch baseline mismatch: ' + name + '\n' +
                                       check.stdout.decode(errors='replace'))
        else:
            print('ALREADY APPLIED: complete shared-RPC upgrade')

        assert patch_command(args.patch, root, shared, reverse=True).returncode == 0
        assert patch_command(args.patch, root, shared).returncode != 0
        print('PASS: complete patch applies, reverse check passes, repeat application rejected')

        if args.bash:
            # Verify both fresh deployment and the user's already-patched state.
            for scenario, reverse_names in [
                ('pristine', ['fairino_shared_rpc.patch', 'fairino_gripper_interface.patch',
                              'fairino_dual_arm_ip.patch']),
                ('existing-gripper', ['fairino_shared_rpc.patch']),
            ]:
                scenario_root = root / scenario
                scenario_package = scenario_root / package.name
                shutil.copytree(package, scenario_package)
                for name in reverse_names:
                    result = patch_command(args.patch, scenario_root, patches / name,
                                           reverse=True, dry=False)
                    assert result.returncode == 0, result.stdout.decode(errors='replace')
                result = subprocess.run(shell[:-1] + [scenario_root.as_posix()], capture_output=True)
                assert result.returncode == 0, result.stdout.decode(errors='replace')
                assert patch_command(args.patch, scenario_root, shared, reverse=True).returncode == 0
                print('PASS: installer scenario', scenario)

            # Deliberately incompatible source: preflight must leave it untouched.
            mismatch_root = root / 'mismatch'
            mismatch_package = mismatch_root / package.name
            shutil.copytree(package, mismatch_package)
            header = mismatch_package / 'include/fairino_hardware/fairino_hardware_interface.hpp'
            header.write_text(header.read_text(encoding='utf-8').replace(
                'class FairinoHardwareInterface:', 'class IncompatibleHardwareInterface:'), encoding='utf-8')
            before = {p.relative_to(mismatch_package): p.read_bytes()
                      for p in mismatch_package.rglob('*') if p.is_file()}
            result = subprocess.run(shell[:-1] + [mismatch_root.as_posix()], capture_output=True)
            after = {p.relative_to(mismatch_package): p.read_bytes()
                     for p in mismatch_package.rglob('*') if p.is_file()}
            assert result.returncode != 0 and before == after
            print('PASS: incompatible source is rejected without modifying original files')

        sources = [package / 'src/fairino_hardware_interface.cpp',
                   package / 'src/fairino_gripper_hardware_interface.cpp']
        flags = [args.cxx, '-std=c++17', '-D_USE_MATH_DEFINES', '-pthread']
        includes = ['-I', tests / 'stubs', '-I', package / 'include',
                    '-I', package / 'include/fairino_hardware']
        executable = root / 'test_shared_rpc.exe'
        run(flags + includes + sources + [tests / 'test_shared_rpc.cpp', '-o', executable])
        run([executable])
        if args.real_sdk_headers:
            # Real SDK headers precede the fake SDK path. No SDK library is linked.
            run(flags + ['-fsyntax-only', '-I', vendor] + includes + sources)
            print('PASS: real vendor SDK header signatures (ROS interfaces are still stubs)')


if __name__ == '__main__':
    main()
