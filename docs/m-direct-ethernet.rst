Run the RSR study with M over direct Ethernet
============================================

This guide describes the setup verified on the lab Pi: Bloom runs in Docker on
the researcher's Mac, M connects directly to that Mac by Ethernet, and the Pi
serves its own face locally. The researcher browser records the interaction.
Automatic analysis still requires the CUDA service described in
:doc:`testing-on-lan`.

Addresses and cables
-------------------

Connect the Ethernet cable directly between M and the Mac's Ethernet adapter.
Wi-Fi can remain connected for internet access.

.. list-table:: Verified example addresses
   :header-rows: 1

   * - Component
     - Address
   * - Mac Ethernet (``en7`` in this setup)
     - ``169.254.180.212``
   * - M Ethernet (``eth0``)
     - ``169.254.189.141``
   * - Researcher interface, on the Mac
     - ``http://localhost:3000/rsr-study``
   * - Bloom backend, as reached from M
     - ``http://169.254.180.212:8080``
   * - Face, opened on M itself
     - ``http://localhost:8765/``

The Mac address is automatically assigned and can change after reconnecting.
Check it every time with ``ipconfig getifaddr en7`` (substitute the actual
Ethernet adapter). Replace the example Mac address in all commands below if it
changes. The earlier ``169.254.189.140`` address is no longer valid for this
verified setup. Do not use the Mac's campus Wi-Fi address for the direct cable.

Start Bloom on the Mac
----------------------

Start Docker Desktop and wait until its engine is ready. From the Bloom repo::

   cd /Users/denio/unr/bloom
   git switch rsr-study-dashboard
   git pull --ff-only
   docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml \
     up -d --build mariadb-dev arsr-db-dev bloom-server-dev bloom-react-dev

If ``docker`` is not found on this Mac, use Docker Desktop's bundled commands::

   export PATH="/Applications/Docker.app/Contents/Resources/bin:/Applications/Docker.app/Contents/Resources/cli-plugins:$PATH"
   docker-compose -f docker-compose.yml -f docker-compose.rsr-test.yml \
     up -d --build mariadb-dev arsr-db-dev bloom-server-dev bloom-react-dev

This starts the interface, recording/archive backend, and databases without
starting the GPU analysis container. Verify the backend::

   curl -f http://localhost:8080/api/rsr-speech/sentences

The frontend build must exclude host ``node_modules``; the feature branch's
``frontend/.dockerignore`` supplies that fix. Do not remove database volumes
when restarting: they hold the issued accounts and study data.

Connect and check M
-------------------

The Pi's existing ``bloom-direct`` NetworkManager profile uses the manual
address above. On the Pi::

   sudo nmcli connection up bloom-direct
   ip -br -4 addr
   curl --interface eth0 --connect-timeout 5 \
     http://169.254.180.212:8080/api/rsr-speech/sentences

Expect sentence data. If setting up a new Pi without that profile::

   sudo nmcli connection add type ethernet ifname eth0 con-name bloom-direct \
     ipv4.method manual ipv4.addresses 169.254.189.141/16 \
     ipv4.never-default yes ipv6.method disabled
   sudo nmcli connection up bloom-direct

The interface is ``eth0``, not ``eht0``. To correct the typo in an existing
profile::

   sudo nmcli connection modify bloom-direct connection.interface-name eth0
   sudo nmcli connection up bloom-direct

For remote administration from the Mac::

   ssh rrl@169.254.189.141

Use the Pi account's issued credentials; passwords are not included here.

Launch the robot on M
--------------------

Stop any existing robot launch before starting another copy. In
``~/.bloom/robot.cfg`` on the Pi, change only the existing backend entry::

   base_url=http://169.254.180.212:8080

Preserve the other robot fields. The speech poller reads this file separately
from the launch argument, so both addresses must match.

The verified Pi has Bloom at ``/home/rrl/development/bloom``. Its robot workspace
also contains an M-SARG checkout at ``robot/src/m-sarg``. Launch from that
workspace::

   source /opt/ros/jazzy/setup.bash
   cd /home/rrl/development/bloom/robot
   source install/local_setup.bash
   ros2 launch bloom_node m_sarg_combined.launch.py \
     base_url:=http://169.254.180.212:8080

Leave the terminal running. Expect:

* ``Session Code:`` followed by a pairing code.
* ``RSR speech node ready`` with the Ethernet backend address.
* ``face web server listening on http://localhost:8765/``.

Use ``m_sarg_combined.launch.py`` for M. ``combined.launch.py`` targets Blossom
hardware and requires ``openhmi_blossom``.

If the M launch file is missing, update Bloom and rebuild before launching::

   cd /home/rrl/development/bloom
   git pull --ff-only
   cd robot
   colcon build --packages-up-to bloom_node
   source install/local_setup.bash

On other installations M-SARG may be a separate workspace: source its
``install/setup.bash`` before Bloom's ``install/local_setup.bash``. Check
``ros2 pkg prefix m_sarg`` to identify the package actually selected by ROS.
An older M-SARG package installed inside Bloom overrides a newer separate
workspace when Bloom is sourced last.

Show the face and pair the study
-------------------------------

On **M's browser**, open ``http://localhost:8765/`` and use **Ctrl+Shift+R** after
updates. The face is served by M, not by the Mac's Bloom backend.

On the **Mac**, open ``http://localhost:3000/rsr-study/login``. Sign in with the
researcher account issued in this Docker backend. The local test login is
``rsr-test-researcher``; obtain its password from the researcher administering
the test. Preview-server accounts belong to a different database.

1. Select **Pair M robot** and enter the current **Session Code** from M's
   terminal or face. Codes can change after a restart; do not reuse a code
   printed in an earlier session or use the robot UUID.
2. Confirm the robot is paired, then create an adult rehearsal session with
   delivery set to **M robot**.
3. Open the child display on the same Mac/browser origin.
4. Start continuous recording and allow microphone access **before** any
   prompts or responses. Use ``localhost`` on the researcher computer.
5. Select **Play prompt**, confirm audible playback from M, then mark the prompt
   ended and open the response window.
6. Finish the rehearsal and replay the saved recording. Confirm it captured
   both the speaker and robot before testing with participants.

Run only one RSR playback consumer. This ROS launch already runs
``rsr_speech_node``; do not also start a browser face that polls Bloom's RSR
queue using a ``robotId`` query parameter.

Troubleshooting the face
------------------------

On the Pi::

   curl -I http://localhost:8765/
   curl -I http://localhost:8765/index.html
   source /opt/ros/jazzy/setup.bash
   source /home/rrl/development/bloom/robot/install/local_setup.bash
   ros2 pkg prefix m_sarg
   ls "$(ros2 pkg prefix m_sarg)/share/m_sarg/web3d/assets"

Both page URLs should return HTTP 200. The current server prevents caching of
both entry URLs. A 403 at ``/`` indicates an old face server; ``/index.html``
is only a temporary workaround, not a replacement for updating the package.

The black-screen investigation found an old July 2D face installed inside
Bloom, shadowing the newer 3D checkout. The Pi was repaired on the nested
M-SARG branch ``fix/m-face-study-connection`` by updating the face server,
browser assets, Wi-Fi helper, and asset installation entries, preserving its
motor configuration. These Pi-specific repairs are not assumed to be available
through a normal remote branch pull. Do not reset that checkout or replace its
motor configuration as a troubleshooting shortcut.

After changing the face source, rebuild the **workspace that owns the selected
package**, then restart the launch::

   cd /home/rrl/development/bloom/robot
   colcon build --packages-select m_sarg
   source install/local_setup.bash

The installed package needs both ``share/m_sarg/web3d`` (including its hashed
JavaScript, WebAssembly, and model files) and ``share/m_sarg/visual_aids``.
The repaired face displays startup failures and reports browser errors to the
``face_node`` terminal. Read those messages rather than repeatedly reinstalling
the browser.

For a remote face check, forward its loopback server from the Mac::

   ssh -N -L 8879:localhost:8765 rrl@169.254.189.141

Then open ``http://localhost:8879/`` on the Mac. Keep the SSH command running.
Rendering there verifies the Pi's server and assets, but the Pi's own browser
and graphics still need an on-device check.

The debug launch used during repair writes to
``/home/rrl/development/bloom/robot/log/m-study-debug.log``. A normal foreground
launch prints to its terminal and stores ROS launch logs under ``~/.ros/log``.
Stop the existing debug launch before using a foreground launch.

Verification
------------

On October 23, 2026, the direct Ethernet backend request, ROS registration,
speech-poller connection, HTTP 200 face entry pages, and browser rendering of
the Pi-served face were verified. The missing-JavaScript diagnostic was also
tested locally. This does not replace an on-device display and audible prompt
rehearsal or validation of CUDA analysis.
