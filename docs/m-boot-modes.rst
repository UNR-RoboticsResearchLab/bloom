Choose M's mode at boot
=======================

On M's automatically logged-in Ubuntu desktop, a startup prompt waits for keyboard
input. Enter one of these values and press Enter:

``dev``
   Leave the Ubuntu desktop available for general development. The managed robot
   stack is stopped; ROS tools can be launched manually.

``auto-rsr``
   Start the M face in kiosk mode, Bloom pairing, and speech/sentence playback.
   The backend is explicitly ``https://dev-bloom.unr.dev``. Open the researcher
   dashboard there and pair using the robot's session code.

   After a sentence and its completion chime, M's face glows green and smiles
   to cue the child's turn to respond. The glow and smile clear when the next prompt
   starts (or the robot loses its backend connection). This is a response cue,
   not a microphone-recording light; the full-session recording remains
   continuous. The researcher still marks response windows in the dashboard.
   The cue is the same after every successfully played sentence, independent
   of the child's answer or score.

Cancel selects development mode. There is no timeout or automatic study launch.
The choice is requested again at each graphical login, including boot. The last
choice is recorded for diagnostics, not silently reused.

Installation
------------

Run as the robot's desktop user, with the Bloom robot workspace already built:

.. code-block:: bash

   cd ~/development/bloom
   bash robot/scripts/install_boot_mode.sh
   bash robot/scripts/select_boot_mode.sh

The selector requires ``zenity`` and ``flock``. It uses the existing Ubuntu
desktop login; it does not change login credentials or enable automatic login.
Install it on an M desktop that already logs in on boot if boot-time prompting
is required. Stop any separately launched robot stack before using these modes
to avoid duplicate nodes.

Switching and troubleshooting
-----------------------------

Open **M robot mode** from Ubuntu's applications menu to choose again. Over SSH,
use these commands as the same desktop user:

.. code-block:: bash

   bash ~/development/bloom/robot/scripts/select_boot_mode.sh dev
   bash ~/development/bloom/robot/scripts/select_boot_mode.sh auto-rsr
   systemctl --user status bloom-auto-rsr.service
   journalctl --user -u bloom-auto-rsr.service -n 60 --no-pager
   cat ~/.local/state/bloom/mode

The user service owns its ROS processes, so switching to development stops the
whole managed stack. It is deliberately not enabled as an independent boot
service: the desktop selector starts it only after AutoRSR is chosen. A failed
stack is restarted after five seconds; inspect its journal if the face or pairing
does not appear. Robot identity continues to live in ``~/.bloom/robot.cfg``.
