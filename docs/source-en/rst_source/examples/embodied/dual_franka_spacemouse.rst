Dual Franka SpaceMouse Collection
=================================

.. figure:: https://raw.githubusercontent.com/RLinf/misc/main/pic/dual-franka.jpg
   :align: center
   :width: 80%
   :alt: Dual-Arm Franka

   Two SpaceMice teleoperating a dual-Franka rig.

Collect dual-arm demonstrations with two SpaceMice, one per arm, on the tcp_rot6d action layout.

Overview
--------

Install the single-machine dual-Franka environment, then teleoperate both arms with two SpaceMice and save demos for downstream SFT.

.. grid:: 2 4 4 4
   :gutter: 2

   .. grid-item-card:: Models
      :text-align: center

      — (teleop collection only)

   .. grid-item-card:: Algorithms
      :text-align: center

      —

   .. grid-item-card:: Tasks
      :text-align: center

      Dual-arm manipulation

   .. grid-item-card:: Hardware
      :text-align: center

      2× Franka · 2 SpaceMice · Robotiq

| **You'll do:** install the ``franka-franky_in_one`` environment → fill in the collect config → collect dual-arm SpaceMouse demos.
| **Prerequisites:** :doc:`dual_franka` (for real-time prerequisites) · two Franka arms · two SpaceMice · Robotiq grippers.

Tasks
~~~~~

.. list-table::
   :header-rows: 1
   :widths: 24 24 24

   * - Task
     - Config / entry point
     - Description
   * - Collection
     - ``realworld_collect_data_dual_franka``
     - Collect dual-arm tcp_rot6d trajectories with two SpaceMice.

Installation
------------

This workflow runs both robots from a single workstation. Install the
``franka-franky_in_one`` environment there:

.. code-block:: bash

   bash requirements/install.sh embodied --env franka-franky_in_one

``franka-franky_in_one`` inherits the original ``franka-franky`` install (the
``franka`` Python extra, the system dependencies, and the prebuilt
``franky-control`` wheel with libfranka bundled) and adds what a single
machine needs:

1. Two catkin workspaces under the venv — ``.venv/franka_catkin_ws1`` and
   ``.venv/franka_catkin_ws2`` — for ROS/serl-based control tooling. Both
   track the same ``LIBFRANKA_VERSION`` (default ``0.19.0``) and are sourced
   from ``.venv/bin/activate``. ``.venv/franka_ws_map.txt`` records which
   robot each workspace belongs to (``ws1`` = left arm, ``ws2`` = right
   arm).
2. The GUI-enabled ``opencv-python`` re-install (lerobot pulls in the
   headless variant).

Keep ``--env franka-franky`` for the two-node rig; its behavior is unchanged.

Pass the robot IPs with ``--left-ip`` / ``--right-ip``. When
``LIBFRANKA_VERSION`` is not set, the install auto-detects the robot system
version through the Desk API of those IPs and picks the matching libfranka
from the official `Franka compatibility matrix
<https://frankarobotics.github.io/docs/compatibility.html>`_: system
``>= 5.9.0`` → libfranka ``0.19.0``, older systems → ``0.15.0``. Export
``LIBFRANKA_VERSION`` to override (avoid libfranka ``0.18.0``):

.. code-block:: bash

   bash requirements/install.sh embodied --env franka-franky_in_one \
       --left-ip 172.16.0.5 --right-ip 172.16.0.2

   export LIBFRANKA_VERSION=0.15.0       # replace with your compatible version
   bash requirements/install.sh embodied --env franka-franky_in_one

The catkin workspaces need ROS Noetic, which is only available on Ubuntu
20.04. Set ``SKIP_ROS=1`` to skip the ROS packages and both workspaces
entirely — the dual-arm runtime drives the robots through the franky wheel
and does not use ROS:

.. code-block:: bash

   SKIP_ROS=1 bash requirements/install.sh embodied --env franka-franky_in_one

Configure the PREEMPT_RT kernel and real-time permissions as described in the
*Installation* section of :doc:`dual_franka`.

Configure the collect config
----------------------------

Edit ``examples/embodiment/config/realworld_collect_data_dual_franka.yaml``.

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - Key
     - What to set
   * - ``left_robot_ip`` / ``right_robot_ip``
     - FCI IP of each arm (``172.16.0.5`` / ``172.16.0.2`` in the shipped
       config).
   * - ``*_camera_serials`` / ``*_camera_type``
     - Camera serial numbers and types per arm, plus one base camera.
   * - ``left_gripper_connection`` / ``right_gripper_connection``
     - Robotiq RS-485 ports as stable ``/dev/serial/by-id/`` paths.
   * - ``left_spacemouse_device_index`` / ``right_spacemouse_device_index``
     - pyspacemouse device indices (defaults: ``0`` / ``1``). If each arm
       responds to the wrong mouse, swap these two values.
   * - ``left_spacemouse_path`` / ``right_spacemouse_path``
     - Optional udev ``by-path`` symlink pinning each mouse to its USB port
       (stable across reboots; wins over the device index).
   * - ``action_scale``
     - SpaceMouse delta gains ``[pos, rot, gripper]`` (default
       ``[1.0, 0.5, 1.0]``).
   * - ``keyboard_reward_wrapper``
     - ``"start_end"`` pedal control (default) or delete the line to end
       episodes by holding ``target_ee_pose``.

Identify the two SpaceMice
~~~~~~~~~~~~~~~~~~~~~~~~~~

The two mice are identical hardware (same VID/PID), so ``device_index`` just
follows the USB enumeration order. Identify which index is which:

.. code-block:: bash

   # List every connected SpaceMouse with its index, hidraw path and udev
   # by-path/by-id symlinks.
   python toolkits/realworld_check/test_spacemouse.py

   # Print live input of one device; wiggle a mouse and note which index
   # lights up.
   python toolkits/realworld_check/test_spacemouse.py --watch 0

If the arms respond to the wrong mouse, swap
``left_spacemouse_device_index`` / ``right_spacemouse_device_index`` in the
config. To make the mapping survive reboots, plug each mouse into a fixed USB
port and set ``left_spacemouse_path`` / ``right_spacemouse_path`` to the
``by-path`` symlinks the listing prints (they override the indices).

Collect demos
-------------

Run collection:

.. code-block:: bash

   bash examples/embodiment/collect_data.sh realworld_collect_data_dual_franka

Both arms start by holding their current pose; moving a SpaceMouse drives its
arm, the left mouse button closes the gripper, the right button opens it.
Untouched arms stay in place.

Episode control, two ways:

- **Pedal (default).** With ``keyboard_reward_wrapper: "start_end"`` the
  3-key pedal maps to: ``a`` start a new episode / abort the current one,
  ``b`` bump ``segment_id``, ``c`` mark success and save the episode.
- **Pose-based.** Delete ``keyboard_reward_wrapper`` and episodes end
  automatically when both arms hold ``target_ee_pose`` for
  ``success_hold_steps`` consecutive steps.

Demos export to ``save_dir`` in LeRobot format with ``robot_type:
"dual_FR3"``.
