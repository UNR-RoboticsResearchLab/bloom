# Testing the robot face without a live backend or robot

`apps/tutorial-fullscreen-face` has a Playwright suite
(`e2e/rsr-face.spec.ts`) that drives the real kiosk face in a real headless
browser through the full poll -> fetch sentence -> play audio -> run the
viseme timeline -> acknowledge cycle. It intercepts the app's HTTP calls at
the network boundary (`page.route()`), so it needs none of the bloom
backend, MariaDB, Azure TTS credentials, or a physical robot — only the
packages built once (see step 4 below) and a browser binary. It asserts on
the sentence/viseme fetches, real audio playback duration, and the
acknowledge round-trip; it does not inspect exact 3D rig weights, since
those live behind an internal, undocumented id scheme in `@vizij/render`'s
store rather than the human-readable path.

```
cd robot/kiosk-face
pnpm install
pnpm --filter @vizij/utils run build
pnpm --filter @vizij/render run build
pnpm --filter @vizij/node-graph-authoring run build
pnpm --filter @vizij/orchestrator-react run build
pnpm --filter @vizij/runtime-react run build
pnpm --filter fullscreen-face exec playwright install chromium   # one-time
pnpm --filter fullscreen-face run test:e2e
```

This is a trade-off worth knowing: it's real coverage of the frontend's
poll/play/animate/acknowledge logic, but it does not exercise the real
ASP.NET controllers or database. For that, use the manual end-to-end flow
below.

## Eyeballing it live: `?robotId=mock`

For manually watching the face speak without any backend, database, Azure
credentials, or robot at all, open the kiosk face with `robotId=mock`:

```
cd robot/kiosk-face
pnpm --filter fullscreen-face run dev
```

Then open `http://localhost:5173/?robotId=mock` and press **N** to speak
the next sentence from `robot/scripts/rsr_sentences.json` (bundled into the
app as `src/mockRsrSentences.ts`, so no fetch is needed). Each press:

- Shows that sentence's text in a subtitle bar at the bottom of the screen.
- Plays a short synthetic silent audio clip and a rotating viseme timeline
  through the real `useStaticVisemePlayback` pipeline, so the mouth cycles
  through its shapes on the same code path real RSR playback uses — there's
  no real phoneme timing behind it, so it's for eyeballing "does playback
  and lip movement happen at all," not for judging accuracy.
- Ignores further N presses until that clip finishes (~350ms/word, clamped
  1.2–4s) and the subtitle clears.

For a real robot id, the subtitle bar shows the actual sentence text too —
`useRsrPolling` fetches `/api/rsr-speech/sentences` once client-side to
resolve each pending command's `sentenceId` to its text (no backend change
needed, since that manifest already carries it).

# Testing the robot face with a real session

Steps to drive the kiosk face app (`apps/tutorial-fullscreen-face`) end to end
against the bloom backend, using the RSR Assessment page's "Robot face"
playback mode.

## 0. Prerequisites

- Backend running and reachable (via `docker-compose up bloom-server-dev
  mariadb-dev` or `dotnet run` from `backend/`). Note the base URL — referred
  to below as `$BACKEND_URL` (e.g. `http://localhost:8080` for the docker
  compose stack, or whatever `dotnet run` prints).
- `robot/tts_module/config.py` filled in with real Azure Speech credentials
  (copy from `config_example.py`).
- pnpm available (`corepack enable` if `pnpm` isn't found).

## 1. Generate sentence audio (one-time)

```
python robot/scripts/generate_rsr_audio.py
```

Writes `backend/wwwroot/rsr-audio/{sentence_NN.<ext>, sentence_NN.visemes.json,
manifest.json}`. Re-run only if the sentence list, voice, or recordings
change.

Confirm it's being served:

```
curl $BACKEND_URL/api/rsr-speech/sentences
```

### Using your own recordings instead of Azure TTS

Drop a file named `sentence_NN.wav` (matching the `id` in `rsr_sentences.json`,
zero-padded — e.g. `sentence_03.wav`) into `robot/scripts/recordings/`. `.flac`
and `.ogg` also work; `.mp3` does not (soundfile can't read it — convert it
first). Re-run the script. Sentences with a matching recording use that audio
file as-is; Azure is still queried for that sentence's text to produce a
viseme timeline, which is time-stretched to match the recording's actual
duration (recordings have no viseme data of their own, so this is an
approximation, not a true alignment). Sentences without a recording fall back
to full Azure synthesis, unchanged. `--recordings-dir` overrides the default
location.

## 2. Register a test robot

```
curl -X POST $BACKEND_URL/api/robot/register \
  -H "Content-Type: application/json" \
  -d '{"name":"Kiosk Test Robot","model":"kiosk-test"}'
```

Save the returned robot `id` — referred to below as `$ROBOT_ID`.

## 3. Create a session for that robot (gets a pairing code)

Session creation requires an authenticated account. Create one if you don't
already have a test account, then log in and create the session, keeping the
cookie jar across both calls:

```
curl -c cookies.txt -X POST $BACKEND_URL/api/user/create \
  -H "Content-Type: application/json" \
  -d '{"fullName":"Test User","email":"kiosktest@example.com","password":"Password123!","selectedRole":"Teacher","userName":"kiosktest"}'

curl -c cookies.txt -b cookies.txt -X POST $BACKEND_URL/api/user/login \
  -H "Content-Type: application/json" \
  -d '{"email":"kiosktest@example.com","password":"Password123!"}'

curl -b cookies.txt -X POST $BACKEND_URL/api/session \
  -H "Content-Type: application/json" \
  -d "{\"robotId\":\"$ROBOT_ID\"}"
```

The session response includes `sessionCode` — a 6-digit pairing code. This is
what gets entered on the RSR Assessment page, not the robot ID itself.

## 4. Start the kiosk face app

```
cd robot/kiosk-face
pnpm install
pnpm --filter @vizij/utils run build
pnpm --filter @vizij/render run build
pnpm --filter @vizij/node-graph-authoring run build
pnpm --filter @vizij/orchestrator-react run build
pnpm --filter @vizij/runtime-react run build
VITE_BLOOM_API_URL=$BACKEND_URL pnpm --filter fullscreen-face run dev
```

Open `http://localhost:5173?robotId=$ROBOT_ID` — this is the face window
that plays audio and animates visemes. Without `robotId` in the URL it just
runs as the plain manual-hotkey demo and never polls for RSR commands.

## 5. Run the assessment

Start the main frontend (`npm start` in `frontend/`), go to
`/rsr-assessment`, fill in age, choose "Robot face" under playback mode,
enter the `sessionCode` from step 3, and click Connect. Start the
assessment and click "Play on robot" on any sentence — the kiosk window
should speak the sentence and animate its mouth in sync.

## Notes

- The kiosk poll interval is 2 seconds (`useRsrPolling.ts`); expect a short
  delay between clicking "Play on robot" and audio starting.
- If the kiosk window logs `Unmapped Azure viseme id`, a viseme ID is
  missing from `azureVisemeMapping.ts` — check the console for which one.
- `packages/@vizij/*` need rebuilding (`pnpm --filter <pkg> run build`)
  after pulling changes to those packages; the app only reads their `dist/`
  output, not source directly.
