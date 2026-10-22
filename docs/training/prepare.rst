1. Prepare the session
======================

What you should be able to do
-----------------------------

Prepare the correct condition, verify the child display and microphone, and
start recording before study instructions or prompts are delivered.

Know the source procedure
-------------------------

The supplied manual describes a 16-sentence recall screening task, identifies
ages 5-9, and gives an approximate administration time of 7-10 minutes. Its
original setup uses recorded auditory stimuli, headphones, and a score sheet.
These are source-manual facts, not evidence that modified robot administration
has the same measurement properties. See :doc:`sources`, manual p. 2.

Before the child arrives
------------------------

* Obtain the current study instructions and condition assignment. Identify which
  device speaks, who advances items, what the researcher may say, and whether
  the study changes any manual procedure. Use :doc:`protocol`.
* Complete a rehearsal with an adult. Confirm that the intended microphone
  captures the adult participant, examiner, and audible stimulus. Bloom records
  the researcher's computer microphone; it does not capture system or robot
  audio directly.
* Confirm the study's permission, assent, and recording procedures are complete
  before research recording begins. Use the study's actual documents and scripts.
* Prepare a participant code, age in completed months, and the assigned cutoff.
  Do not choose a different cutoff after seeing a score.
* Have the manual, approved introduction, approved practice material, and session
  documentation available. The child should not see the researcher dashboard.

Open the correct interface
--------------------------

1. Open the deployed Bloom study URL ending in ``/rsr-study``. The GitHub Pages
   documentation site is a training reference, not the live screening app.
2. Sign in using the researcher login issued by the study administrator.
3. Confirm the researcher name shown in the dashboard.
4. Enter the participant code, age, cutoff, and assigned condition.
5. For M delivery, use **Pair M robot** and confirm the correct physical robot.
   Confirm audible playback in an adult rehearsal; pairing alone does not verify
   that M's speech software is running.
6. Select **Create session**, then **Open child display**. Move the child window
   to the child's display and confirm **Child display connected**. Both windows
   must use the same browser profile and origin on the same computer.

Start recording before the interaction
--------------------------------------

Select **Start continuous recording**, grant the requested microphone access,
and confirm **Continuous recording is running** before the approved introduction,
practice, or first test prompt. Keep the researcher tab open for the interaction.

Do not assume that creating a session, opening the child window, or marking a
response starts audio capture. Only the continuous-recording control starts it.

If the child speaks before a prompt, leave recording running. This early speech
belongs to the interaction record even if it is outside an analysis window.

.. note::

   Microphone access requires HTTPS or localhost. During the adult rehearsal,
   finish and listen to a test recording to check the actual audio. A status
   indicator by itself cannot establish that the chosen microphone is audible.

Knowledge check
---------------

Before the researcher reads the introduction, which state must be visible?
**Continuous recording is running.** The child window should also be connected,
but opening it does not start recording.
