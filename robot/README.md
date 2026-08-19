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
|-- stt_module/                    # Speech-to-text engine (Azure / Whisper)
|-- tts_module/                    # Text-to-speech engine (Azure)
|-- llm_module/                    # Language model engine (Azure OpenAI / OpenAI)
|-- face_display/                  # Standalone demo's browser face (blossom_face.html)
|
|-- src/                           # ROS2 workspace packages
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

`stt_module/`, `tts_module/`, and `llm_module/` are not tied to either run
mode — they're plain Python libraries with a pluggable engine interface
(`engine_interface.py`) and Azure implementations (`engine_azure.py`,
`engine_azure_openai.py`), plus a Whisper/OpenAI fallback. Both
`conversation_orchestrator.py` and the ROS2 `bloom_speech` nodes
(`stt_node.py`, `tts_node.py`, `llm_node.py`) import directly from these
modules — the ROS2 nodes just add `robot/` to `sys.path` and reuse the same
engines and `config.py` files. Editing an engine here affects both run
modes. `bloom_speech` also ships `stt_node_vosk.py`, an offline Vosk-based
STT alternative that doesn't depend on this module.

## Standalone Demo

### 1. Install Dependencies
```bash
pip install -r requirements.txt
```
Installs pygame, the Azure Speech SDK, Azure OpenAI, and audio libraries.
Each of `stt_module/`, `tts_module/`, `llm_module/` also has its own
`requirements.txt` if you're using a module in isolation.

### 2. Configure API Keys
```bash
# STT
cp stt_module/config_example.py stt_module/config.py
# TTS
cp tts_module/config_example.py tts_module/config.py
# LLM
cp llm_module/config_example.py llm_module/config.py
# Edit each with your Azure credentials
```
See each module's README for where to get API keys.

### 3. Set Up Microphone (if needed)
```bash
cd stt_module
python setup_mic.py
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
1. **Listen**: Azure STT transcribes your speech
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
cd stt_module && python test_engine.py
cd tts_module && python test_engine.py
cd llm_module && python test_engine.py
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

Two credential mechanisms exist side by side:
- `config.py` files (per `stt_module`/`tts_module`/`llm_module`, copied
  from `config_example.py`) — used by the shared engines regardless of run
  mode.
- `.env` (copy from `.env.example`) — Azure OpenAI/TTS credentials loaded
  as environment variables; referenced by `bloom.service` via
  `EnvironmentFile`.

Both need to be filled in for the ROS2 stack to talk to Azure.

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
- Run `stt_module/setup_mic.py`
- Test with `stt_module/test_engine.py`

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
