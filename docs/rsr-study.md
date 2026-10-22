# RSR study workspace

New researchers: complete [researcher training](training/index.rst) for
manual-based administration, scoring, adult rehearsal, and readiness review.
This page is the software and operations reference.

Open `/rsr-study` to enter the study. Without a verified researcher session,
Bloom redirects to `/rsr-study/login`. Researchers enter the **assigned login and
password provided by the study administrator**, then go directly to the study
control room. The study login has no public signup option. Identity cookies are
checked on the server before study controls open; cached browser profile data
alone does not grant access.

## Issue a researcher login

A Bloom administrator signs in, opens `/rsr-study/accounts` (or selects **Issue
researcher login** in the study dashboard), and supplies a name, unique login,
and password. Give those credentials and the study login URL to the researcher.
The account receives the existing `Researcher` Identity role. Passwords use
Bloom's Identity hashing, validation, and login lockout rules; they are not stored
in the frontend or documentation. There are no built-in researcher credentials.
Public registration cannot create researcher or administrator accounts. Existing
administrators may also enter the study with their Bloom username and password.
An SLP account needs an assigned Researcher role to access the study interface.

**Sign out of study** ends the server session and clears the cached profile. It
is disabled during recording; finish recording first. Password resets are handled
by the study administrator using the deployment's account-management process.

Study authentication endpoints:

- `POST /api/rsr-study/auth/login`: assigned `login` and `password`.
- `GET /api/rsr-study/auth/session`: verifies the authenticated researcher/admin.
- `POST /api/rsr-study/auth/logout`: clears the Identity authentication cookie.
- `POST /api/rsr-study/auth/researchers`: administrator-only issuance of a
  `login`, `password`, and optional `fullName`.

The child display has no researcher credentials or administrative data. It remains
a separate window opened by the signed-in researcher.

This workspace is separate from the older `/rsr-assessment` and `/arsr/study`
screens. It reuses Bloom's robot pairing context, sentence manifest, RSR robot
speech queue, and AutoRSR analysis API. No database migration is needed.

## Protocol setup

The actual three conditions have not yet been specified. Setup therefore offers
three configurable labels and an explicit delivery setting for each. The initial
examiner/browser/M mapping is provisional, not a research protocol. Condition
assignment is manual; the condition definitions and selected assignment are frozen
in each session's log. This implementation does not randomize or counterbalance.

The complete 16-item manifest must be available at
`GET /api/rsr-speech/sentences`. Generate the existing RSR audio assets with
`robot/scripts/generate_rsr_audio.py` if needed. For M, pair through Bloom's existing
pairing card before creating the session. M must be running the existing kiosk
RSR speech polling/viseme playback integration; a paired robot alone does not
establish that its speech software is running.

## Administration

1. Enter a participant code, age in months, cutoff, and assigned condition.
2. Create the session and open the child display. Move that window to the child's
   monitor. Both windows must use the same browser profile and origin on the same
   computer. The dashboard reports whether the child display is connected.
3. Start continuous recording **before any instructions or prompts**. This captures
   the examiner computer's microphone. Position it to capture the child, examiner,
   and M. Recording begins only after the browser grants microphone access.
4. Cue/play an item. For examiner delivery, the examiner then reads it aloud.
   Browser delivery uses the pre-generated stimulus audio. Robot delivery queues
   the sentence through Bloom's existing RSR speech API.
5. After the audible prompt ends, mark the response window, then end the attempt.
   Browser playback must finish before the response-start control is available.
   Robot and examiner response timing is explicitly confirmed by the administrator:
   a successful queue request is not treated as audible playback completion.
6. Select another item, or use a manual-authorized repeat by starting a new attempt. All attempts
   remain in the log. The latest completed, non-excluded response window per item
   is used for automatic analysis. Record a reason before excluding an attempt.
7. Break/resume controls change the child display while **keeping audio recording**.
8. Finish recording, review the audio, correct response boundaries if necessary,
   and run automatic analysis. Boundary corrections and exclusions are locked once
   analysis completes, keeping its source windows stable. Failed analysis can be
   retried with the same full recording; it never starts a replacement recording.
9. Enter reviewed scores with a reason. Original automatic scores, the automated
   decision, previous override values, and reasons remain in the event log. A
   reviewed total is not a recalculated screening decision. Missing item scores
   remain unscored, rather than being displayed as reviewed zeroes.
10. Save the archive to Bloom. After further review/overrides, save again to create
    another immutable revision. Also download the full audio and JSON log as needed.

The child screen shows item number, listening/response/break cues, and a completion
screen. It does not expose written target sentences, transcripts, scores, or
administrator controls. Displaying the written sentence would change a recall task.

## Recording and recovery

One `MediaRecorder` runs from recording start until session finish, including speech
before prompts, between items, and during breaks. Item/response timestamps are
annotations on that recording, not recording start/stop commands. They use the
recorder's monotonic clock. Original automatic results and the exact analysis item
selection are retained alongside reviewed scores.

Each one-second audio chunk and session state are saved to IndexedDB. The final
recording waits for the recorder's `stop` event and final `dataavailable` chunk;
there is no guessed timeout. Microphone tracks are released on finish or unmount.
Local write failures are shown, with an in-memory recording retained for download.
The event log uses recording-relative milliseconds plus UTC wall-clock audit times.

A reload or microphone interruption ends continuous capture. The saved session can
be recovered for review/export, but cannot silently resume as if no gap occurred.
A browser/process crash can lose the last unflushed chunk, and recovery/playability
of partial files depends on the browser's recording container. Close/reload prompts
warn while recording; navigating away still stops capture. Keep the admin tab open
through the entire interaction. Recordings are audio only, not video or direct
robot/system-audio loopback. Capturing a robot prompt acoustically requires an
appropriate microphone position.

## Archive storage and access

`POST /api/rsr-study/archives/{studyId}` accepts stopped-session metadata and the
complete audio recording (up to 200 MB total request). It atomically publishes a
new revision after both files finish writing. Revisions are never overwritten.
`GET /api/rsr-study/archives` returns the authenticated examiner's latest revisions.
`GET /api/rsr-study/archives/{studyId}/{revision}/audio` retrieves that owner's audio.
Researcher/admin authorization and per-owner storage apply to every archive endpoint.
An archive is not accessible to another examiner merely because they know its ID.

The default directory is `backend/App_Data/rsr-study`, outside `wwwroot` and ignored
by Git. Configure `RsrStudy__ArchivePath` to change it. Both Compose configurations
mount a dedicated `rsr_study_archives` volume at `/var/rsr-study`; retain/back up
this volume. Non-Docker deployments should put the archive outside any directory
replaced by deployment scripts. No archive is uploaded until **Save archive to
Bloom** is selected. Browser-local recovery is scoped in the UI to the signed-in
examiner; it is not a substitute for saving the server archive.

Microphone access requires HTTPS or localhost. Opening Bloom over plain HTTP at a
LAN IP will not provide recording APIs in normal browser configurations.

## Verification

- Frontend: `cd frontend && npm test -- --watchAll=false --runInBand`
- Build: `cd frontend && npm run build`
- Backend: `dotnet test tests/bloom.Tests.csproj --filter 'FullyQualifiedName~RsrStudyControllerTests|FullyQualifiedName~RsrSpeechServiceTests'`
- Browser: serve `frontend/build` with SPA fallback, install Playwright/Chromium in
  a local test environment, then run `node tests/frontend/rsrStudy.browser.cjs`.
  `BLOOM_URL` defaults to `http://127.0.0.1:3097`; `BLOOM_PLAYWRIGHT_PATH` can point
  to an existing Playwright package. The browser check uses a synthetic microphone
  and mocked API responses; actual capture, IndexedDB, and child windows are real.

Physical M playback, microphone placement, and the study's final three condition
protocols still require an operator/hardware rehearsal before participant use.
