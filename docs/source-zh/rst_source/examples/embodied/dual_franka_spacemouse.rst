双 Franka 空间鼠标采集
================================================

.. figure:: https://raw.githubusercontent.com/RLinf/misc/main/pic/dual-franka.jpg
   :align: center
   :width: 80%
   :alt: 双 Franka

   两个空间鼠标分别遥操作双 Franka rig 的两条手臂。

使用两个空间鼠标（每个控制一条手臂）采集 tcp_rot6d 动作布局的双臂示教数据。

概览
----------------------------------------

安装单机 dual-Franka 环境，用两个空间鼠标遥操作双臂并保存示教数据，供后续 SFT 使用。

.. grid:: 2 4 4 4
   :gutter: 2

   .. grid-item-card:: 模型
      :text-align: center

      —（仅遥操作采集）

   .. grid-item-card:: 算法
      :text-align: center

      —

   .. grid-item-card:: 任务
      :text-align: center

      Dual-arm manipulation

   .. grid-item-card:: 硬件
      :text-align: center

      2× Franka · 2 SpaceMice · Robotiq

| **你要做：** 安装 ``franka-franky_in_one`` 环境 → 填写采集配置 → 用双空间鼠标采集双臂示教数据。
| **前置条件：** :doc:`dual_franka`（实时性前置配置）· 两台 Franka · 两个空间鼠标 · Robotiq 夹爪。

任务
~~~~~~~~~~~~~~~~~~~~

.. list-table::
   :header-rows: 1
   :widths: 24 24 24

   * - 任务
     - 配置 / 入口
     - 说明
   * - 数据采集
     - ``realworld_collect_data_dual_franka``
     - 用两个空间鼠标采集双臂 tcp_rot6d 轨迹。

安装
----------------------------------------

本流程在一台工作站上同时控制两台机器人，安装 ``franka-franky_in_one``
环境：

.. code-block:: bash

   bash requirements/install.sh embodied --env franka-franky_in_one

``franka-franky_in_one`` 继承了原有 ``franka-franky`` 的安装内容（``franka``
Python extra、系统依赖、内置 libfranka 的预编译 ``franky-control`` wheel），
并补充单机所需的：

1. 在 venv 下编译两个 catkin 工作空间 —— ``.venv/franka_catkin_ws1`` 与
   ``.venv/franka_catkin_ws2`` —— 供 ROS/serl 控制工具链使用。两者都遵循
   相同的 ``LIBFRANKA_VERSION``（默认 ``0.19.0``），并在
   ``.venv/bin/activate`` 中被 source。``.venv/franka_ws_map.txt`` 记录了
   每个工作空间对应的机器人（``ws1`` = 左臂、``ws2`` = 右臂）。
2. 重装 GUI 版 ``opencv-python``（lerobot 会引入 headless 版本）。

双节点方案继续使用 ``--env franka-franky``，其原有行为不变。

通过 ``--left-ip`` / ``--right-ip`` 传入机器人 IP。未设置
``LIBFRANKA_VERSION`` 时，安装会自动通过这些 IP 的 Desk API 探测机器人
系统版本，并按官方 `Franka 兼容性矩阵
<https://frankarobotics.github.io/docs/compatibility.html>`_ 自动选择匹配的
libfranka：系统版本 ``>= 5.9.0`` → libfranka ``0.19.0``，更旧的系统 →
``0.15.0``。显式导出 ``LIBFRANKA_VERSION`` 可覆盖（避免使用 libfranka
``0.18.0``）：

.. code-block:: bash

   bash requirements/install.sh embodied --env franka-franky_in_one \
       --left-ip 172.16.0.5 --right-ip 172.16.0.2

   export LIBFRANKA_VERSION=0.15.0       # 替换为你的兼容版本
   bash requirements/install.sh embodied --env franka-franky_in_one

catkin 工作空间依赖 ROS Noetic，仅 Ubuntu 20.04 提供。设置 ``SKIP_ROS=1``
可跳过 ROS 系统包与两个工作空间 —— 双臂运行时通过 franky wheel 驱动机器
人，不依赖 ROS：

.. code-block:: bash

   SKIP_ROS=1 bash requirements/install.sh embodied --env franka-franky_in_one

按照 :doc:`dual_franka` 的 *安装* 一节配置 PREEMPT_RT 内核与实时权限。

配置采集文件
----------------------------------------

编辑 ``examples/embodiment/config/realworld_collect_data_dual_franka.yaml``。

.. list-table::
   :header-rows: 1
   :widths: 30 70

   * - 配置项
     - 设置内容
   * - ``left_robot_ip`` / ``right_robot_ip``
     - 各手臂的 FCI IP（随附配置中为 ``172.16.0.5`` / ``172.16.0.2``）。
   * - ``*_camera_serials`` / ``*_camera_type``
     - 每臂相机的序列号与类型，外加一台 base 相机。
   * - ``left_gripper_connection`` / ``right_gripper_connection``
     - Robotiq RS-485 串口，使用稳定的 ``/dev/serial/by-id/`` 路径。
   * - ``left_spacemouse_device_index`` / ``right_spacemouse_device_index``
     - pyspacemouse 设备索引（默认 ``0`` / ``1``）。若手臂响应了错误的
       鼠标，交换这两个值即可。
   * - ``left_spacemouse_path`` / ``right_spacemouse_path``
     - 可选的 udev ``by-path`` 符号链接，把每个鼠标固定到其 USB 端口
       （重启后依然稳定；优先于设备索引）。
   * - ``action_scale``
     - 空间鼠标增量增益 ``[位置, 旋转, 夹爪]``（默认 ``[1.0, 0.5, 1.0]``）。
   * - ``keyboard_reward_wrapper``
     - 默认 ``"start_end"`` 踏板控制；删除该行则改为双臂保持
       ``target_ee_pose`` 自动结束回合。

识别两个空间鼠标
~~~~~~~~~~~~~~~~~~~~

两个鼠标是相同硬件（VID/PID 相同），``device_index`` 只按 USB 枚举顺序
排列。用下面的命令识别哪个索引是哪个鼠标：

.. code-block:: bash

   # 列出所有已连接的空间鼠标：索引、hidraw 路径与 udev by-path/by-id 链接
   python toolkits/realworld_check/test_spacemouse.py

   # 实时打印某个设备的输入；晃动一个鼠标，看哪个索引有数据
   python toolkits/realworld_check/test_spacemouse.py --watch 0

若手臂响应了错误的鼠标，交换配置里的 ``left_spacemouse_device_index`` /
``right_spacemouse_device_index``。要让映射在重启后依然有效，把每个鼠标
固定插到某个 USB 口，并将列出的 ``by-path`` 链接填入
``left_spacemouse_path`` / ``right_spacemouse_path``（其优先级高于索引）。

采集示教数据
----------------------------------------

启动采集：

.. code-block:: bash

   bash examples/embodiment/collect_data.sh realworld_collect_data_dual_franka

启动后双臂保持当前位姿；移动某个空间鼠标即驱动对应手臂，鼠标左键闭合
夹爪、右键张开夹爪，未操作的手臂原地保持。

回合控制有两种方式：

- **踏板（默认）。** ``keyboard_reward_wrapper: "start_end"`` 时三键踏板
  映射为：``a`` 开始新回合 / 中止当前回合，``b`` 递增 ``segment_id``，
  ``c`` 标记成功并保存回合。
- **位姿判定。** 删除 ``keyboard_reward_wrapper`` 后，双臂在
  ``target_ee_pose`` 保持 ``success_hold_steps`` 个连续步即自动结束回合。

示教数据以 LeRobot 格式导出到 ``save_dir``，``robot_type`` 为
``"dual_FR3"``。
