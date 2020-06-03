Building MPS Voxels
===================

This page describes the steps necessary to build the ``mps_voxels`` project.
It assumes you have cloned the package into a suitable ROS workspace, taken here to be ``~/catkin_ws/src/mps_voxels``.

Clone ROS Sources
-----------------

Dependencies that are not installable through the ``rosdep`` system are listed in the ``.rosinstall`` file.
Use the ``wstool`` utility to clone those sources into a catkin workspace.

.. code-block:: bash

    # From mps_voxels root directory
    cp .rosinstall ~/catkin_ws/src/
    wstool status && wstool up

Install dependencies through rosdep
-----------------------------------

.. literalinclude:: ../ci/Dockerfile
    :language: bash
    :caption:
    :lines: 76-78
    :dedent: 4

Building Additional Packages from Source
----------------------------------------

.. literalinclude:: ../ci/install_cgal.sh
    :language: bash
    :caption:
    :lines: 5-

.. literalinclude:: ../ci/install_octomap.sh
    :language: bash
    :caption:
    :lines: 5-

.. literalinclude:: ../ci/install_opencv.sh
    :language: bash
    :caption:
    :lines: 5-

.. literalinclude:: ../ci/add_local_paths.sh
    :language: bash
    :caption:
    :lines: 7-
