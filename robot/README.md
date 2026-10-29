# Bloom Conversation Robot

Robot-side software for Bloom, a speech-therapy robot that has natural
conversations with children using Azure AI services. This directory holds
**two run modes** that share the same speech engines:

| | Standalone demo | ROS2 stack |
|---|---|---|
| Entry point | `conversation_orchestrator.py` | `ros2 launch bloom_node combined.launch.py` |
| Needs ROS2? | No | Yes (Jazzy) |
| Needs the Bloom backend? | No | Yes, for lessons/RSR |
| Face | `face_display/blossom_face.html` (browser) | `bloom_face` (pygame) + `kiosk-face` (browser, vizij) |
| Use for | Quick local testing of the STT→LLM→TTS→face loop | The physical robot |

If you just want to see/hear Bloom talk on a laptop, start with
[Standalone Demo](#standalone-demo). If you're working on the physical
robot or the lesson/RSR-assessment integration with the backend, skip to
[ROS2 Stack](#ros2-stack).

## Project Structure
```
robot/
|-- conversation_orchestrator.py   # Standalone demo entry point
|-- test_viseme_sync.py            # Test lip-sync timing
|-- requirements.txt               # Deps for the standalone demo
|-- bloom.service                  # systemd unit -> ROS2 combined launch
|-- robot.json / .env.example      # Robot id / Azure credentials (env form)
|
|-- stt_module/                    # DEFUNCT: not in git; the demo still imports it (see below)
|-- face_display/                  # Standalone demo's browser face (blossom_face.html)
|
|-- src/                           # ROS2 workspace packages
|   |-- bloom_speech_engines/      # TTS + LLM engines (plain Python package, no ROS imports)
|   |-- bloom_node/                # C++: web service client, state machine, lesson/behavior coordination
|   |-- bloom_speech/              # Python: STT/TTS/LLM ROS2 nodes + RSR sentence playback
|   |-- bloom_face/                # Python: pygame face node (emotions, visemes, visual aids)
|   |-- bloom_face_bridge/         # Python: bridges bloom_face topics to the browser face
|   `-- bloom_interface/           # Reserved for future shared ROS msg/srv definitions (currently empty)
|
|-- kiosk-face/                    # vizij-based browser face (pnpm workspace), new face renderer
`-- scripts/                       # Lesson loading, RSR audio generation, kiosk launcher
```

Each module/package has its own README with more detail.

## Shared Speech Engines

The `bloom_speech_engines` package (`src/bloom_speech_engines/`, with `tts` and
`llm` subpackages) is not tied to either run mode — it's a plain Python library with a pluggable engine interface
(`engine_interface.py`) and Azure implementations (`engine_azure.py`,
`engine_azure_openai.py`), plus a Whisper/OpenAI fallback. Both
`conversation_orchestrator.py` and the ROS2 `bloom_speech` nodes
(`stt_node.py`, `tts_node.py`, `llm_node.py`) import directly from these
package — colcon builds it for the ROS2 stack, and `pip install -e` makes it
importable for the standalone demo. Editing an engine here affects both run
modes. `bloom_speech` also ships `stt_node_vosk.py`, an offline Vosk-based
STT alternative that doesn't depend on this module.

## Standalone Demo

### 1. Install Dependencies
```bash
pip install -r requirements.txt
pip install -e 'src/bloom_speech_engines[azure,llm]'
```
The first installs pygame, the Azure Speech SDK, Azure OpenAI, and audio libraries; the second makes
`bloom_speech_engines` importable (the `azure`/`llm` extras pull in just the SDKs the engines need).

> **The standalone demo is currently broken.** `conversation_orchestrator.py` imports a
> `stt_module` that no longer exists (it is defunct and was removed from git). The demo will not
> start until its speech-to-text step is replaced or removed. The ROS2 stack is unaffected: its
> `stt_node` calls the Azure SDK directly. Ignore every `stt_module` step below.

### 2. Configure API Keys
```bash
# TTS
cp src/bloom_speech_engines/bloom_speech_engines/tts/config_example.py src/bloom_speech_engines/bloom_speech_engines/tts/config.py
# LLM
cp src/bloom_speech_engines/bloom_speech_engines/llm/config_example.py src/bloom_speech_engines/bloom_speech_engines/llm/config.py
# Edit each with your Azure credentials
```
See each module's README for where to get API keys.

### 3. Set Up Microphone (if needed)
```bash
# stt_module/setup_mic.py -- DEFUNCT (stt_module no longer exists)
```

### 4. Run It
```bash
python conversation_orchestrator.py
```
What happens:
1. `face_display/blossom_face.html` opens in your browser, served from a
   local HTTP server on `localhost:8000`.
2. Bloom says "Hi! I'm Bloom..."
3. Speak when you see "Listening..."
4. Bloom responds with an animated, lip-synced face.
5. Press `Ctrl+C` to stop.

**Flow:**
```
Speech → STT → LLM → TTS → Face
              ↑
         (maintains history)
```
1. **Listen**: speech-to-text (the defunct `stt_module`; see the note above)
2. **Think**: GPT generates a response based on conversation history
3. **Speak**: Azure TTS creates audio + viseme (mouth-shape) data
4. **Animate**: the face shows emotions and syncs its mouth to the visemes

**Communication:** Python writes to `face_display/face_control.json`; the
browser polls it every 100ms.

**Bloom's personality:** the system prompt in `conversation_orchestrator.py`
defines patient, encouraging behavior with short (2-3 sentence) responses
and follow-up questions — edit it there to change how Bloom talks.

### Testing Individual Modules
```bash
python -m bloom_speech_engines.tts.live_check    # real Azure call, needs tts/config.py
python -m bloom_speech_engines.llm.live_check    # real LLM call, needs llm/config.py
python -m pytest src/bloom_speech_engines/tests  # offline unit tests
cd face_display && python test_face.py
python test_viseme_sync.py
```

## ROS2 Stack

The production robot runs as a ROS2 (Jazzy) workspace rooted at `robot/`
(packages in `src/`, build output in `install/`/`build/`), integrated with
the Bloom backend for lessons and RSR (repeated-sentence) speech
assessments.

### Packages (`src/`)

- **bloom_node** (C++) — web service client to the Bloom backend, robot
  state machine, priority-based behavior coordination, and lesson/feedback
  polling. See [src/bloom_node/README.md](src/bloom_node/README.md).
- **bloom_speech** (Python) — `stt_node`/`stt_node_vosk` (speech-to-text,
  Azure or offline Vosk), `tts_node`, `llm_node` (reuse the shared engines
  above), and `rsr_speech_node` (polls the backend for RSR playback
  commands and plays pre-generated sentence audio).
- **bloom_face** (Python) — fullscreen pygame face driven by ROS topics:
  emotions, viseme-based lip sync, and lesson visual aids
  (`visual_aids/`).
- **bloom_face_bridge** (Python) — subscribes to the same topics as
  `bloom_face` and re-publishes them over a local WebSocket/HTTP server
  (ports 8765/8766) so the browser-based `kiosk-face` app can drive the
  same face in parallel, without touching any existing publisher.
- **bloom_interface** — reserved for shared ROS message/service
  definitions; currently just a placeholder package.

The robot also depends on `openhmi_blossom` (the Blossom hardware driver)
for motor control — it's an external ROS2 package, not part of this repo.

### Build
```bash
cd robot
colcon build
source install/local_setup.bash
```

### Run
```bash
# Everything: bloom_node, bloom_speech, bloom_face, bloom_face_bridge, and
# the openhmi_blossom motor driver
ros2 launch bloom_node combined.launch.py

# Or just the web-service/behavior node
ros2 launch bloom_node bloom.launch.py base_url:=http://localhost:5000
```
`combined.launch.py` starts everything except the browser kiosk face
(`kiosk-face`), which is a GUI process, not a ROS node — launch it
separately (see below) once you want to validate it alongside the pygame
face.

### kiosk-face (vizij browser face)

`kiosk-face/` is a pnpm workspace containing the newer, browser-based face
renderer (`apps/tutorial-fullscreen-face`, built on `packages/@vizij/*`) —
the eventual replacement for `bloom_face`'s pygame window. It runs
alongside `bloom_face`/`bloom_face_bridge`, not instead of them, until
validated on real hardware.

```bash
cd robot/kiosk-face
pnpm install
pnpm --filter @vizij/utils run build
pnpm --filter @vizij/render run build
pnpm --filter @vizij/node-graph-authoring run build
pnpm --filter @vizij/orchestrator-react run build
pnpm --filter @vizij/runtime-react run build

# Watch it live against a real robot/session, or ?robotId=mock with no backend at all
pnpm --filter fullscreen-face run dev
```
On the robot itself, `scripts/launch_face_kiosk.sh` builds the app and
opens it full-screen in kiosk-mode Chromium, using this robot's id from
`~/.bloom/robot.cfg`. See [kiosk-face/README.md](kiosk-face/README.md) for
the full mock/live/E2E testing flows, including how to drive an RSR
assessment end to end against the backend.

### Utility Scripts (`scripts/`)

- `generate_rsr_audio.py` — one-time generation of RSR sentence audio +
  viseme timelines via Azure TTS (or your own recordings), written to
  `backend/wwwroot/rsr-audio/`.
- `load_lesson.py` — publishes a lesson JSON file to `/load_lesson`
  (defaults to `src/bloom_node/config/homophones.json`).
- `launch_face_kiosk.sh` — builds and opens the vizij kiosk face (see
  above).
- `rsr_sentences.json` — the RSR sentence list consumed by
  `generate_rsr_audio.py` and mirrored into the kiosk app for mock mode.

### Configuration

Two credential mechanisms exist side by side. Which one applies depends on the run mode; the
engines in `bloom_speech_engines` read neither themselves, they take credentials as constructor
arguments.

**1. `config.py` files: standalone demo and `live_check` scripts only.**
Each is copied from the `config_example.py` beside it and is git-ignored (`robot/.gitignore`):

| File (under `src/bloom_speech_engines/bloom_speech_engines/`) | Keys |
|---|---|
| `tts/config.py` | `AZURE_SUBSCRIPTION_KEY`, `AZURE_REGION`, `AZURE_DEFAULT_VOICE` |
| `llm/config.py` | `AZURE_OPENAI_KEY`, `AZURE_OPENAI_ENDPOINT`, `AZURE_OPENAI_DEPLOYMENT`, `AZURE_OPENAI_API_VERSION`; optional `OPENAI_API_KEY`, `OPENAI_MODEL` |

Read by `conversation_orchestrator.py` (`from bloom_speech_engines.tts import config`) and by
`python -m bloom_speech_engines.{tts,llm}.live_check`. Because they live inside the package
directory, they only work with an editable install (`pip install -e`); never ship them in a built
package or `.deb`. The ROS2 nodes never read them.

**2. `.env`: the ROS2 stack.** Copy `.env.example` to `robot/.env` on a dev machine; on a robot the file
is `/etc/bloom/robot.env` (`root:root`, mode `0600`, created at provisioning). systemd loads it via
`EnvironmentFile=`, so the nodes just see environment variables. Outside systemd, point
`BLOOM_ENV_FILE` at the file and the `bloom_speech` nodes load it themselves
(`bloom_speech/envfile.py`; variables already set always win). **No package ships this file**: it is
not in `bloom_speech`'s install, so `.deb` upgrades and rollbacks never overwrite or restore secrets.
It maps onto the same settings under different names:

| Setting | `config.py` key | `.env` variable | Read by |
|---|---|---|---|
| TTS key | `AZURE_SUBSCRIPTION_KEY` | `AZURE_TTS_KEY` | `tts_node`, and `stt_node` (it reuses the TTS key) |
| TTS region | `AZURE_REGION` | `AZURE_TTS_REGION` (default `westus2`) | same |
| TTS voice | `AZURE_DEFAULT_VOICE` | `AZURE_TTS_VOICE` (default `en-US-AvaNeural`) | `tts_node` |
| LLM key / endpoint | `AZURE_OPENAI_KEY` / `AZURE_OPENAI_ENDPOINT` | same names | `llm_node` |
| LLM deployment / API version | `AZURE_OPENAI_DEPLOYMENT` / `AZURE_OPENAI_API_VERSION` | same names (defaults `gpt-4.1-mini`, `2024-02-15-preview`) | `llm_node` |
| Backend login | n/a | `BLOOM_ROBOT_ID`, `BLOOM_ROBOT_SECRET` | `bloom_node` (issued by an admin; robots no longer self-register) |

Both need to be filled in if you use both run modes. For the ROS2 stack alone, only `.env` matters.

### Remote shell agent (`agent/`)

`robot/agent/` is `robot-agent`, which gives admins a shell on this robot from the fleet
dashboard over an outbound TLS tunnel. It's plain CMake rather than a colcon package
(`COLCON_IGNORE`), so it keeps working when the ROS stack doesn't, and it has its own systemd
unit (`bloom-agent.service`). See [agent/README.md](agent/README.md).

### Systemd Service

`bloom.service` runs the full ROS2 stack (`combined.launch.py`) on boot —
it does **not** run the standalone demo.
```bash
sudo cp robot/bloom.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable bloom
sudo systemctl start bloom
```

## Troubleshooting

**Face doesn't open (standalone demo):**
- Try manually: http://localhost:8000/blossom_face.html
- Check terminal for HTTP server errors

**No audio:**
- Verify pygame is installed
- Check speaker volume
- Verify TTS config

**Microphone issues:**
- Check OS permissions (System Settings → Privacy)
- (`stt_module/setup_mic.py` and its `test_engine.py` are defunct.) For the ROS2 `stt_node`, check the mic device with `arecord -l`.

**LLM errors:**
- Verify API key and deployment name
- Check Azure quota/credits

**Face not responding (standalone demo):**
- Make sure it was opened via HTTP (not `file://`)
- Check the browser console (F12)
- Verify `face_display/face_control.json` exists

**kiosk-face logs `Unmapped Azure viseme id`:**
- A viseme ID is missing from `azureVisemeMapping.ts` in
  `kiosk-face/apps/tutorial-fullscreen-face/src/` — check the console for
  which one.

## Credits

`face_display/` is based on work by Denielle Oliva:
https://github.com/denielleoliva/blsm_unr

Uses Azure AI services (Speech-to-Text, Text-to-Speech, OpenAI).
