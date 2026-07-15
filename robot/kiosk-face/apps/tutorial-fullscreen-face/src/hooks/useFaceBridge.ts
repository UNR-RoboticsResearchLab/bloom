import { useEffect, useRef, useState } from "react";
import type { PoseRigConfig } from "@vizij/runtime-react";
import { useFaceEmotion } from "./useFaceEmotion";
import { useLiveVisemePlayback, type LiveVisemeEvent } from "./useLiveVisemePlayback";

const RECONNECT_DELAY_MS = 2000;

const WS_URL = (import.meta.env.VITE_FACE_BRIDGE_WS_URL as string | undefined) ?? "ws://localhost:8765";
const HTTP_URL = (import.meta.env.VITE_FACE_BRIDGE_HTTP_URL as string | undefined) ?? "http://localhost:8766";

// Mirrors face_node.py's on_face_state emotion_map (bloom_node/bloom_speech
// send higher-level state words here, not always a raw emotion name).
const FACE_STATE_TO_EMOTION: Record<string, string> = {
  smile: "happy",
  neutral: "neutral",
  sad: "sad",
  excited: "excited",
  thinking: "thinking",
};

// Mirrors face_node.py's on_robot_state state_emotion_map. Unmapped states
// are ignored there too (no pass-through), so this does the same.
const ROBOT_STATE_TO_EMOTION: Record<string, string> = {
  waiting: "calm",
  talking: "happy",
  loading: "thinking",
  idle: "calm",
};

export type VisualAidState = {
  images: string[];
  labels: string[];
  footers: string[];
} | null;

type BridgeEvent = { topic: string; data: unknown };

export function resolveVisualAidImageUrl(filename: string): string {
  return `${HTTP_URL.replace(/\/$/, "")}/${filename}`;
}

export function useFaceBridge(poseConfig: PoseRigConfig | null, enabled: boolean) {
  const { setEmotion, suppressForViseme } = useFaceEmotion(poseConfig);
  const { setTimeline, start, stop } = useLiveVisemePlayback(suppressForViseme);

  const [visualAid, setVisualAid] = useState<VisualAidState>(null);
  const [micActive, setMicActive] = useState(false);
  const [pairingCode, setPairingCode] = useState<string | null>(null);

  const setEmotionRef = useRef(setEmotion);
  setEmotionRef.current = setEmotion;
  const setTimelineRef = useRef(setTimeline);
  setTimelineRef.current = setTimeline;
  const startRef = useRef(start);
  startRef.current = start;
  const stopRef = useRef(stop);
  stopRef.current = stop;

  useEffect(() => {
    if (!enabled) return undefined;
    let cancelled = false;
    let socket: WebSocket | null = null;
    let reconnectTimer = 0;

    function handleEvent({ topic, data }: BridgeEvent) {
      switch (topic) {
        case "face_emotion":
          if (typeof data === "string") setEmotionRef.current(data);
          break;
        case "robot/face_state":
          if (typeof data === "string") {
            const emotion = FACE_STATE_TO_EMOTION[data.toLowerCase()] ?? data.toLowerCase();
            setEmotionRef.current(emotion);
          }
          break;
        case "robot/state":
          if (typeof data === "string") {
            const emotion = ROBOT_STATE_TO_EMOTION[data.toLowerCase()];
            if (emotion) setEmotionRef.current(emotion);
          }
          break;
        case "/face/visemes":
          if (Array.isArray(data)) setTimelineRef.current(data as LiveVisemeEvent[]);
          break;
        case "/face/command": {
          const cmd = data as { command?: string; emotion?: string } | null;
          if (cmd?.command === "start_audio_sync") startRef.current();
          else if (cmd?.command === "stop_audio_sync") stopRef.current();
          else if (cmd?.command === "set_emotion" && cmd.emotion) setEmotionRef.current(cmd.emotion);
          break;
        }
        case "/face/visual_aid": {
          const payload = data as { command?: string; images?: string[]; labels?: string[]; footers?: string[] } | null;
          if (payload?.command === "hide") {
            setVisualAid(null);
          } else if (payload) {
            setVisualAid({
              images: (payload.images ?? []).slice(0, 2),
              labels: payload.labels ?? [],
              footers: payload.footers ?? [],
            });
          }
          break;
        }
        case "/robot/session_code":
          setPairingCode(String(data));
          break;
        case "/stt/enable":
          setMicActive(data === true || data === "true");
          break;
        default:
          break;
      }
    }

    function connect() {
      if (cancelled) return;
      socket = new WebSocket(WS_URL);

      socket.onmessage = (event) => {
        try {
          handleEvent(JSON.parse(event.data as string));
        } catch (err) {
          console.error("[face] Failed to parse bridge event", err);
        }
      };

      socket.onclose = () => {
        if (cancelled) return;
        reconnectTimer = window.setTimeout(connect, RECONNECT_DELAY_MS);
      };

      socket.onerror = () => {
        socket?.close();
      };
    }

    connect();

    return () => {
      cancelled = true;
      window.clearTimeout(reconnectTimer);
      socket?.close();
    };
  }, [enabled]);

  return { visualAid, micActive, pairingCode };
}
