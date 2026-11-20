"""Azure voice adapter with Bloom face visemes; no physical motor commands."""
import queue
import json
import math
import subprocess
import threading
import time


def viseme_timeline(events):
    """Include Vizij's milliseconds and pygame's legacy 100 ns offsets."""
    timeline = []
    for event in events or []:
        try:
            milliseconds = float(event.timestamp_ms)
            shape = int(event.viseme_id)
            if math.isfinite(milliseconds) and milliseconds >= 0 and 0 <= shape <= 21:
                timeline.append({"viseme_id": shape,
                                 "timestamp_ms": milliseconds,
                                 "audio_offset": round(milliseconds * 10000)})
        except (AttributeError, TypeError, ValueError, OverflowError):
            continue
    timeline.sort(key=lambda item: item["audio_offset"])
    if timeline and (timeline[0]["audio_offset"] > 0 or timeline[0]["viseme_id"] != 0):
        # The existing face advances by inspecting the next timeline entry.
        # Start from a closed mouth so the first real shape isn't skipped.
        timeline.insert(0, {"viseme_id": 0, "timestamp_ms": 0, "audio_offset": 0})
    return timeline


class SpeechWorker:
    """One worker owns the SDK; callbacks enqueue events for the ROS thread."""
    def __init__(self, adapter, player=subprocess.Popen, face_emit=None, face_prepare_seconds=0.05):
        self.adapter = adapter
        self.player = player
        self.face_emit = face_emit
        self.face_prepare_seconds = face_prepare_seconds
        self.commands = queue.Queue()
        self.events = queue.Queue()
        self.interrupted = threading.Event()
        self.shutdown_requested = threading.Event()
        self.desired_mic = False
        self.recognizer = None
        self.bindings = []
        self.generation = 0
        self.speaking = False
        self.outstanding_speech = False
        self.thread = threading.Thread(target=self.run, daemon=True)

    def enable(self, enabled):
        self.desired_mic = enabled
        if not enabled:
            self.generation += 1  # Discard callbacks from a previous listening window.
        self.commands.put(("mic", enabled))

    def speak(self, text):
        self.outstanding_speech = True
        self.commands.put(("speak", text))

    def interrupt(self):
        if self.outstanding_speech:
            self.interrupted.set()

    def close(self):
        self.shutdown_requested.set()
        self.desired_mic = False
        self.interrupted.set()
        self.commands.put(("shutdown", None))

    def stop_mic(self):
        recognizer, self.recognizer = self.recognizer, None
        self.generation += 1
        if recognizer is not None:
            try:
                recognizer.stop_continuous_recognition_async().get()
            finally:
                for signal, callback in self.bindings:
                    signal.disconnect_all()
                self.bindings = []

    def start_mic(self):
        if self.recognizer is not None or not self.desired_mic or self.speaking:
            return
        sdk = self.adapter.sdk
        self.generation += 1
        generation = self.generation
        recognizer = sdk.SpeechRecognizer(
            speech_config=self.adapter.speech_config,
            audio_config=sdk.audio.AudioConfig(use_default_microphone=True),
        )

        def push(kind, value=""):
            self.events.put((kind, generation, value))

        def partial(evt):
            if evt.result.text.strip():
                push("partial", evt.result.text.strip())

        def final(evt):
            if evt.result.reason == sdk.ResultReason.RecognizedSpeech:
                push("final", evt.result.text.strip())
            else:
                push("no_match")

        def canceled(evt):
            if evt.reason == sdk.CancellationReason.Error:
                push("recognition_error", "Azure microphone recognition was canceled. Check credentials, network, and audio input.")

        self.bindings = [
            (recognizer.speech_start_detected, lambda evt: push("speech_start")),
            (recognizer.recognizing, partial),
            (recognizer.recognized, final),
            (recognizer.canceled, canceled),
            (recognizer.session_stopped, lambda evt: push("recognition_error", "Microphone recognition ended unexpectedly.")),
        ]
        self.recognizer = recognizer
        for signal, callback in self.bindings:
            signal.connect(callback)
        recognizer.start_continuous_recognition_async().get()
        if self.desired_mic and generation == self.generation:
            push("listening")

    def play(self, text):
        self.speaking = True
        process = None
        try:
            self.stop_mic()
            if self.interrupted.is_set() or self.shutdown_requested.is_set():
                return
            self.events.put(("speaking", None, text))
            result = self.adapter.tts.synthesize(text=text, include_viseme=self.face_emit is not None)
            if not result.metrics.success or not result.audio_filepath:
                raise RuntimeError("Azure speech synthesis did not produce audio.")
            if self.interrupted.is_set() or self.shutdown_requested.is_set():
                return
            timeline = viseme_timeline(getattr(result, "viseme_events", []))
            if self.face_emit is not None and timeline:
                # Wait until the ROS thread publishes the new timeline. A short
                # lead-in gives the local face a chance to receive it before sync.
                prepared = self.emit_face("prepare", timeline)
                if prepared:
                    if self.interrupted.wait(self.face_prepare_seconds) or self.shutdown_requested.is_set():
                        return
                    self.emit_face("start", None)
            elif self.face_emit is not None:
                self.events.put(("face_warning", None, "No usable visemes returned; playing audio without mouth animation."))
            if self.interrupted.is_set() or self.shutdown_requested.is_set():
                return
            process = self.player(["paplay", result.audio_filepath])
            while process.poll() is None:
                if self.interrupted.is_set() or self.shutdown_requested.is_set():
                    process.terminate()
                    try:
                        process.wait(timeout=2)
                    except subprocess.TimeoutExpired:
                        process.kill()
                        process.wait(timeout=2)
                    break
                time.sleep(0.02)
            if process.returncode and not self.interrupted.is_set() and not self.shutdown_requested.is_set():
                raise RuntimeError("paplay could not play the generated speech.")
        except Exception:
            # Publish an error before draining the outstanding speech acknowledgement.
            # Avoid copying exception text that might contain credential-bearing URLs.
            self.desired_mic = False
            self.events.put(("error", None, "Speech generation/playback failed. Check Azure Speech settings and PulseAudio."))
        finally:
            if self.face_emit is not None:
                # Complete/interrupt/failure all restore the mouth before the
                # controller receives its playback acknowledgement.
                self.emit_face("stop", None)
            self.speaking = False
            self.outstanding_speech = False
            self.interrupted.clear()
            self.events.put(("done", None, "done"))

    def emit_face(self, kind, data):
        try:
            return self.face_emit(kind, data)
        except Exception:
            self.events.put(("face_warning", None, "Face output failed; continuing audio without animation."))
            return False

    def current(self, generation):
        return (generation == self.generation and self.desired_mic
                and not self.speaking and not self.shutdown_requested.is_set())

    def run(self):
        try:
            while not self.shutdown_requested.is_set():
                command, value = self.commands.get()
                if command == "shutdown":
                    break
                try:
                    if command == "mic":
                        if value and self.desired_mic:
                            self.start_mic()
                        else:
                            self.stop_mic()
                    elif command == "speak":
                        self.play(value)
                except Exception:
                    self.desired_mic = False
                    self.events.put(("error", None, "Could not start/stop microphone recognition. Check Azure Speech settings and default input."))
                    try:
                        self.stop_mic()
                    except Exception:
                        pass
        finally:
            try:
                self.stop_mic()
            except Exception:
                pass


def main(args=None):
    import rclpy
    from rclpy.node import Node
    from std_msgs.msg import String
    from .storytelling_controller_node import activity_helpers

    class SpeechNode(Node):
        def __init__(self):
            super().__init__("storytelling_speech")
            self.declare_parameter("voice", "en-US-AvaNeural")
            self.declare_parameter("segmentation_silence_ms", 700)
            self.declare_parameter("enable_face", True)
            # This is the same tested credential/engine loader as the standalone activity.
            adapter = activity_helpers().VoiceAdapter(self.get_parameter("voice").value)
            adapter.speech_config.set_property(
                adapter.sdk.PropertyId.Speech_SegmentationSilenceTimeoutMs,
                str(self.get_parameter("segmentation_silence_ms").value),
            )
            self.face_enabled = self.get_parameter("enable_face").value
            self.visemes_pub = self.create_publisher(String, "/face/visemes", 10)
            self.face_command_pub = self.create_publisher(String, "/face/command", 10)
            self.worker = SpeechWorker(adapter, face_emit=self.queue_face if self.face_enabled else None)
            # One ordered topic prevents a late partial from overtaking its final
            # transcript or a failed playback's drain acknowledgement.
            self.event_pub = self.create_publisher(String, "/study/speech_event", 100)
            self.create_subscription(String, "/stt/enable", self.on_enable, 10)
            self.create_subscription(String, "/tts/speak", lambda msg: self.worker.speak(msg.data), 10)
            self.create_subscription(String, "/tts/interrupt", lambda msg: self.worker.interrupt(), 10)
            self.create_timer(0.02, self.pump)
            self.worker.thread.start()
            self.get_logger().info("Speech ready, microphone off. Voice: " + adapter.selected_voice)

        def send(self, publisher, text):
            if not rclpy.ok():
                return
            message = String()
            message.data = text
            try:
                publisher.publish(message)
            except Exception:
                if rclpy.ok():
                    raise

        def face_command(self, command, **settings):
            payload = {"command": command, **settings}
            self.send(self.face_command_pub, json.dumps(payload))

        def queue_face(self, kind, data):
            if self.worker.shutdown_requested.is_set() or not rclpy.ok():
                return False
            packet = {"kind": kind, "data": data, "ready": threading.Event(), "ok": False,
                      "canceled": threading.Event()}
            self.worker.events.put(("face", None, packet))
            # Never publish ROS messages from the SDK/audio worker thread.
            if not packet["ready"].wait(timeout=1.0):
                packet["canceled"].set()
                return False
            return packet["ok"]

        def dispatch_face(self, packet):
            try:
                if packet["canceled"].is_set() or not rclpy.ok():
                    return

                if packet["kind"] == "prepare":
                    self.face_command("stop_audio_sync")
                    self.face_command("set_mode", mode="emotion")
                    self.face_command("set_emotion", emotion="happy")
                    self.send(
                        self.visemes_pub,
                        json.dumps(packet["data"]),
                    )

                elif packet["kind"] == "start":
                    if (
                        not self.worker.interrupted.is_set()
                        and not self.worker.shutdown_requested.is_set()
                    ):
                        self.face_command("set_mode", mode="viseme")
                        self.face_command("start_audio_sync")

                elif packet["kind"] == "stop":
                    self.face_command("stop_audio_sync")
                    self.face_command("set_mode", mode="emotion")
                    self.face_command("set_emotion", emotion="happy")

                packet["ok"] = True

            except Exception:
                self.get_logger().warning(
                    "Could not publish face animation commands."
                )
            finally:
                packet["ready"].set()

        def on_enable(self, msg):
            self.worker.enable(msg.data.strip().lower() == "true")

        def pump(self):
            # Drain the entire queue in order, then the controller's next timer
            # can see partial activity before making another silence decision.
            while True:
                try:
                    kind, generation, value = self.worker.events.get_nowait()
                except queue.Empty:
                    return
                if generation is not None and not self.worker.current(generation):
                    continue
                if kind == "face":
                    self.dispatch_face(value)
                elif kind == "face_warning":
                    self.get_logger().warning(value)
                elif kind == "done":
                    self.send_event("done")
                elif kind in {"error", "recognition_error"}:
                    self.worker.enable(False)
                    self.get_logger().error(value)
                    self.send_event("error", value)
                elif kind in {"speech_start", "partial", "no_match"}:
                    self.send_event(kind)
                elif kind == "final":
                    if value:
                        self.get_logger().info("[Recognized] " + value)
                        self.send_event("final", value)
                    else:
                        self.send_event("no_match")
                elif kind == "listening":
                    self.get_logger().info("Listening... You can speak now.")
                elif kind == "speaking":
                    self.get_logger().info("[Robot] " + value)

        def send_event(self, kind, text=""):
            self.send(self.event_pub, json.dumps({"kind": kind, "text": text}))

    rclpy.init(args=args)
    node = None
    try:
        node = SpeechNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            if node.face_enabled and rclpy.ok():
                node.face_command("stop_audio_sync")
            node.worker.close()
            node.worker.thread.join(timeout=3)
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()