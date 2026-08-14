# Copyright 2026 The RLinf Authors.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

"""Identify which SpaceMouse is which on a multi-SpaceMouse machine.

Two identical SpaceMice share the same VID/PID, so ``device_index`` (used by
``DualSpacemouseIntervention``) just follows the HID enumeration order, which
can swap between reboots. This tool:

1. Lists every connected SpaceMouse with its pyspacemouse device index, its
   hidraw path, and any udev ``by-path``/``by-id`` symlinks. The ``by-path``
   link is pinned to the physical USB port — use it as
   ``left_spacemouse_path`` / ``right_spacemouse_path`` in the collect config
   so the mapping survives reboots.
2. With ``--watch <index>``, prints live input of one device for a few
   seconds. Wiggle one mouse, note which index lights up, and assign it to
   the arm it should drive.

Usage::

    python toolkits/realworld_check/test_spacemouse.py            # list
    python toolkits/realworld_check/test_spacemouse.py --watch 0  # identify
"""

from __future__ import annotations

import argparse
import glob
import os
import time

_SPACEMOUSE_VID = 0x256F  # 3Dconnexion


def _enumerate_spacemice() -> list[dict]:
    """Return connected SpaceMice in HID enumeration order (matches
    ``pyspacemouse.open(device_index=...)`` selection)."""
    from easyhid import Enumeration

    devices = []
    for dev in Enumeration().find():
        if dev.vendor_id == _SPACEMOUSE_VID:
            devices.append(
                {
                    "path": dev.path,
                    "product": dev.product_string or "unknown",
                }
            )
    return devices


def _udev_links(realpath_target: str) -> list[str]:
    links = []
    for pattern in (
        "/dev/hidraw/by-path/*",
        "/dev/hidraw/by-id/*",
        "/dev/input/by-path/*",
        "/dev/input/by-id/*",
    ):
        for link in glob.glob(pattern):
            try:
                if os.path.realpath(link) == realpath_target:
                    links.append(link)
            except OSError:
                continue
    return links


def cmd_list() -> None:
    devices = _enumerate_spacemice()
    if not devices:
        print("No SpaceMouse (3Dconnexion) devices found.")
        return
    print(f"Found {len(devices)} SpaceMouse device(s):\n")
    for index, dev in enumerate(devices):
        print(f"device_index {index}: {dev['product']} ({dev['path']})")
        for link in _udev_links(dev["path"]):
            print(f"    by-path/by-id: {link}")
    print(
        "\nPin each mouse to a USB port and put the by-path link into "
        "left_spacemouse_path / right_spacemouse_path in the collect config."
    )


def cmd_watch(index: int, seconds: float) -> None:
    from rlinf.envs.realworld.common.spacemouse.spacemouse_expert import (
        SpaceMouseExpert,
    )

    expert = SpaceMouseExpert(device_index=index)
    deadline = time.time() + seconds
    print(f"Watching device_index {index} for {seconds:g}s — wiggle the mouse.")
    while time.time() < deadline:
        action, buttons = expert.get_action()
        print(f"  action={action.round(3)} buttons={buttons}")
        time.sleep(0.2)


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Identify SpaceMouse devices for dual-arm teleop."
    )
    parser.add_argument(
        "--watch",
        type=int,
        metavar="INDEX",
        help="Print live input of device INDEX to identify which mouse it is.",
    )
    parser.add_argument(
        "--seconds", type=float, default=10.0, help="Duration for --watch."
    )
    args = parser.parse_args()

    if args.watch is not None:
        cmd_watch(args.watch, args.seconds)
    else:
        cmd_list()


if __name__ == "__main__":
    main()
