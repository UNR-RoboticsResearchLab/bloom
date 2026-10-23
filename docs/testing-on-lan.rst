Test AutoRSR locally with M on the same network
===============================================

Run Bloom on the researcher's computer and point the robot at that computer's
LAN address. The browser and robot use different URLs for the same backend.
These instructions describe the ``rsr-study-dashboard`` branch and use
``192.168.1.100`` as an example researcher-computer address. Replace it with the
actual address; it is not the robot's address.

.. list-table:: Example addresses
   :header-rows: 1
   :widths: 35 65

   * - Component
     - Address
   * - Researcher browser, on the Bloom computer
     - ``http://localhost:3000/rsr-study/login``
   * - Backend, from the same computer
     - ``http://localhost:8080``
   * - Backend, from M
     - ``http://192.168.1.100:8080``
   * - AutoRSR, when running in the same Compose network
     - ``http://auto_rsr:5050`` from the backend container

``localhost`` on M means M itself. Do not use it for a backend running on your
laptop. Conversely, use ``localhost`` for the researcher browser so microphone
recording works without configuring LAN HTTPS. Keep the child window in the same
browser profile and origin as the researcher page.

Prerequisites
-------------

* The computer and M are on a network that permits device-to-device connections.
  Guest Wi-Fi or campus client isolation can prevent this even on the same SSID.
* Docker Desktop or Docker Engine with Compose is installed and running on the
  Bloom computer. Port 8080 is reachable from M. Ports 3000 and 8080 are free.
* M has its normal ROS2 Jazzy, ``bloom`` workspace, motor driver, audio device,
  and existing robot configuration. These instructions do not replace robot
  installation; see :doc:`robot setup <robot-guide>`.
* For automatic transcription, the current AutoRSR implementation needs a
  supported NVIDIA/CUDA machine. Its service loads WhisperX on CUDA with
  ``float16``; it does not currently select CPU or Apple GPU automatically.
  A Mac can run Bloom and control M, with AutoRSR on a separate GPU machine.

The existing local preview used port 3097 and a separate SQLite test database.
That preview is not the Docker stack described here. Its researcher account does
not automatically exist in the Docker database. Do not run two backends on 8080.

1. Prepare the Bloom checkout
-----------------------------

Run on the researcher computer, from the directory containing the repositories::

   cd bloom
   git switch rsr-study-dashboard
   git pull --ff-only

The expected sibling layout for full local analysis is::

   work/
     bloom/
       docker-compose.yml
       docker-compose.rsr-test.yml
     AutoRsr-NoBert-SARG/
       Dockerfile
       arsr_microservice.py
       english.json

The added Compose override corrects two current defaults: the base file expects
an ``AutoRSR`` folder inside Bloom, and Development settings point analysis at
``localhost:5050``, which would mean the backend container itself. The override
uses the sibling checkout and ``http://auto_rsr:5050``.

An alternate AutoRSR checkout can be selected with ``AUTO_RSR_SOURCE``. Compose
resolves relative paths from the base Compose file. See the official
`Compose override documentation <https://docs.docker.com/compose/how-tos/multiple-compose-files/merge/>`_.

2. Start Bloom on the computer
------------------------------

Choose one of the following analysis arrangements. Run the commands from the
Bloom repository root. Use the same ``-f`` options in later Compose commands.

A. Bloom and AutoRSR on the same NVIDIA/CUDA computer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

With NVIDIA container support already configured, start the entire stack::

   docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml up -d --build

The base AutoRSR service requests an NVIDIA GPU. Deleting that request alone does
not make the current CUDA-only Python service run on a Mac or CPU-only computer.

B. Bloom on a Mac or another computer; AutoRSR on a separate GPU machine
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

On the GPU machine, use the AutoRSR repository and its configured Python/CUDA
environment. Install its dependencies and FFmpeg if not already installed, then
start the Bloom-compatible service from that repository's root::

   cd AutoRsr-NoBert-SARG
   ARSR_PORT=5050 python3 arsr_microservice.py

This entry point exposes ``POST /api/analyze``. The older standalone UI/API entry
point has a different request contract; do not substitute it for this service.
The repository's pinned dependencies and CUDA requirements must be satisfied;
this guide does not claim the GPU environment has been installed or tested.

For example, if the GPU machine is ``192.168.1.50``, start only Bloom and its
required databases on the researcher computer::

   RSR_ANALYSIS_URL=http://192.168.1.50:5050 \
     docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml \
     up -d --build mariadb-dev arsr-db-dev bloom-server-dev bloom-react-dev

The explicit service list leaves out the local AutoRSR container and its GPU
reservation. The GPU service's port 5050 must be reachable from the Bloom backend.
Use the equivalent environment-variable assignment for your shell on Windows.

C. Test recording and robot delivery before enabling automatic scoring
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

Start only Bloom and its databases::

   docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml \
     up -d --build mariadb-dev arsr-db-dev bloom-server-dev bloom-react-dev

You can test researcher access, recording, M playback, review, and archives.
**Analyze response windows** requires a working AutoRSR service and will fail
until one is connected. You do not need AutoRSR merely to play the pre-generated
sentences on M.

3. Check the computer and its LAN address
-----------------------------------------

Wait for backend startup and migrations. Inspect status and logs if needed::

   docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml ps
   docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml logs --tail=80 bloom-server-dev

Confirm the 16-item manifest and a stimulus file are served::

   curl -f http://localhost:8080/api/rsr-speech/sentences
   curl -I http://localhost:8080/rsr-audio/sentence_01.wav

The audio/viseme assets are already included in this branch. If your checkout is
missing them, follow the audio-generation procedure in
:doc:`face playback testing </robot/kiosk-face/README>` before rebuilding the backend.

Find the address of the computer's active LAN adapter:

* macOS: ``ipconfig getifaddr en0``; use the actual active adapter if different.
* Linux: ``hostname -I``; select the address on M's network.
* Windows: ``ipconfig``; use the active Wi-Fi/Ethernet adapter's IPv4 address.

If an old preview already occupies 8080, stop that preview using its own launch
terminal before starting Docker. Do not stop an unrelated process solely because
it appears in a port listing.

4. Point M at the computer
--------------------------

On M, first check reachability. Substitute the computer's LAN address::

   curl -f http://192.168.1.100:8080/api/rsr-speech/sentences
   curl -I http://192.168.1.100:8080/rsr-audio/sentence_01.wav

If these fail, resolve network routing, the host firewall's 8080 rule, or guest
network isolation before debugging pairing. M does not need database access.

With the robot stack stopped, set only the ``base_url`` entry in
``~/.bloom/robot.cfg`` to::

   base_url=http://192.168.1.100:8080

Preserve the other robot configuration fields and record the previous backend
address if you will switch back. The RSR speech node reads this file separately
from the C++ node's launch argument, so both must identify the same backend.

On M, update/build the appropriate checkout using your normal robot deployment
procedure. From the robot workspace, a standard source build is::

   source /opt/ros/jazzy/setup.bash
   cd /path/to/bloom/robot
   colcon build
   source install/local_setup.bash
   ros2 launch bloom_node combined.launch.py base_url:=http://192.168.1.100:8080

Use the robot's existing hardware/serial launch arguments where needed. Do not
start a second copy if its service is already running. Watch for successful
registration/login, a pairing code, and the RSR speech node reporting readiness
with the correct ``robot_id`` and ``base_url``.

Choose one RSR playback consumer
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

The combined ROS launch already starts ``rsr_speech_node``. It polls the backend,
plays sentence audio on the robot, and publishes face/viseme messages.

For this test, use that ROS speech path. If using the browser face alongside
it, open the face without a ``robotId`` query parameter so it can use the local
face bridge without also polling the RSR speech queue. Do not use the default
``launch_face_kiosk.sh`` unchanged for this arrangement: it adds ``robotId``.

Alternatively, the browser face can own RSR playback with
``VITE_BLOOM_API_URL=http://192.168.1.100:8080`` and ``?robotId=<actual-id>``, but
then do not run the ROS RSR speech poller for that robot. Two pollers can play or
acknowledge the same command. Use the existing
:doc:`kiosk setup guide </robot/kiosk-face/README>` for its package build commands.

5. Sign in and pair from the researcher computer
------------------------------------------------

Open **http://localhost:3000/rsr-study/login** in the researcher's browser.
Use a researcher account issued in this backend's database. An administrator
can create one at ``/rsr-study/accounts`` after signing in.

In the study dashboard:

1. Select **Pair M robot** and enter the pairing code produced by M. Use the
   pairing code, not the robot's UUID. Check the name of the device you paired.
2. Use a clearly marked adult test participant code, age in months, and cutoff.
3. Select the rehearsal condition and set its delivery to **M robot**. The three
   editable defaults are not approved study-condition definitions.
4. Select **Create session**, then **Open child display**. Keep both windows on
   this computer and the same origin; move the child window to another monitor
   if available. It need not be hosted by the robot.
5. Select **Start continuous recording** and confirm capture before speaking or
   playing the first item. Use an adult rehearsal, not a participant, to verify setup.
6. Select **Play prompt**. Hear the sentence on M. Only after audible completion
   select **Prompt ended / start response window**; after the final response,
   select **End attempt**.
7. Finish the test recording and play it back. Confirm it contains both the adult
   and M. The microphone is on the researcher computer; it must acoustically
   capture the robot too.
8. If AutoRSR is configured, analyze the marked response windows. Review results,
   then save and retrieve the archive using the same researcher account.

If a face display exposes written test sentences as subtitles, treat that as a
software-test display and resolve it against the intended study condition before
participant use. See :doc:`researcher training <training/index>` for administration
fidelity and the supplied manual's rules.

The normal recording page must be localhost or HTTPS. Plain
``http://192.168.1.100:3000`` is useful for reachability checks but normally lacks
browser microphone access. See
`getUserMedia secure-context requirements <https://developer.mozilla.org/en-US/docs/Web/API/MediaDevices/getUserMedia>`_.

Troubleshooting
---------------

.. list-table::
   :header-rows: 1
   :widths: 35 65

   * - Symptom
     - Check
   * - Robot cannot connect
     - Its backend URL must contain the researcher's LAN IP and port 8080. Test the manifest URL from M, not just from the computer.
   * - Microphone is unavailable
     - Open the researcher page using localhost on the computer, check browser permissions, and confirm the correct microphone.
   * - Robot is paired but silent
     - Confirm a real RSR playback consumer is running, its configuration points to this backend, and audio files load from M.
   * - Duplicate speech or unexpectedly cleared commands
     - Check for both the ROS RSR node and a browser face polling the same robot ID. Use one consumer.
   * - Analysis is unavailable
     - The backend container needs the configured AutoRSR URL; localhost inside it is not a host or sibling container.
   * - AutoRSR health is OK but analysis fails
     - ``/health`` is a liveness check. WhisperX loads lazily on the first analysis, so check CUDA/model/dependency availability and the service logs.
   * - Earlier test login does not work
     - Confirm which database/backend is running. The separate SQLite preview account is not automatically copied into MariaDB.

Use ``docker compose -f docker-compose.yml -f docker-compose.rsr-test.yml down``
to stop this Compose stack after testing. Keep its named volumes if you need the
test accounts or archives; do not add ``-v`` unless you intend to delete that data.

Verification limits
-------------------

The Compose override is checked with Compose's configuration renderer. These
instructions were checked against the branch's launch files and playback code;
a full NVIDIA container installation, LAN, and physical-M rehearsal still need
to be tested in your environment.
