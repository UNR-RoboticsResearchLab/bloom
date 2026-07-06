import { useCallback, useEffect, useRef, useState } from "react";
import { useStaticVisemePlayback } from "./useStaticVisemePlayback";
import { MOCK_RSR_SENTENCES } from "../mockRsrSentences";
import {
  buildMockVisemeTimelineBlobUrl,
  buildSilentWavBlobUrl,
  estimateSpeechDurationMs,
} from "./mockAudio";

const POLL_INTERVAL_MS = 2000;
const MOCK_ROBOT_ID = "mock";
const MOCK_ADVANCE_KEY = "n";

type PendingSpeechResponse = {
  commandId: string;
  sentenceId: number;
  audioUrl: string;
  visemeUrl: string;
};

type ActiveSpeech = PendingSpeechResponse & { text: string | null };

function resolveUrl(base: string, path: string): string {
  if (/^https?:\/\//i.test(path)) return path;
  return `${base.replace(/\/$/, "")}${path}`;
}

// Drives the kiosk face's speech playback. The kiosk browser is opened with
// ?robotId=<id> baked into the launch URL:
//   - absent: no-op, so the plain hotkey-demo behavior of this app is
//     unaffected when not used for RSR administration.
//   - "mock": press N to advance through a local, bundled sentence list with
//     synthetic silent audio + a rotating viseme timeline — no bloom
//     backend or Azure TTS needed, for testing the face standalone.
//   - anything else: treated as a real robot id, polls the bloom backend
//     for a queued RSR sentence to speak.
export function useRsrPolling(): { activeText: string | null } {
  const robotIdRef = useRef<string | null>(null);
  if (robotIdRef.current === null) {
    robotIdRef.current = new URLSearchParams(window.location.search).get("robotId");
  }
  const robotId = robotIdRef.current;
  const isMock = robotId === MOCK_ROBOT_ID;
  const apiBaseUrl = (import.meta.env.VITE_BLOOM_API_URL as string | undefined) ?? "http://localhost:5000";

  const [active, setActive] = useState<ActiveSpeech | null>(null);
  const playingRef = useRef(false);
  const activeUrlsRef = useRef<{ audioUrl: string; visemeUrl: string } | null>(null);

  // Real mode: fetch the sentence manifest once so pending commands (which
  // only carry a sentenceId) can be displayed with their text.
  const sentenceTextRef = useRef<Map<number, string> | null>(null);
  useEffect(() => {
    if (!robotId || isMock) return;
    let cancelled = false;
    fetch(`${apiBaseUrl}/api/rsr-speech/sentences`)
      .then((res) => res.json())
      .then((entries: { id: number; text: string }[]) => {
        if (cancelled) return;
        sentenceTextRef.current = new Map(entries.map((e) => [e.id, e.text]));
      })
      .catch((err) => {
        console.error("[face] Failed to load RSR sentence manifest", err);
      });
    return () => {
      cancelled = true;
    };
  }, [robotId, isMock, apiBaseUrl]);

  // Real mode: poll for a queued sentence.
  useEffect(() => {
    if (!robotId || isMock) return undefined;
    let cancelled = false;

    async function poll() {
      if (playingRef.current || cancelled) return;
      try {
        const res = await fetch(`${apiBaseUrl}/api/rsr-speech/${robotId}/pending`);
        if (res.status !== 200) return;
        const data = (await res.json()) as PendingSpeechResponse;
        playingRef.current = true;
        setActive({
          ...data,
          audioUrl: resolveUrl(apiBaseUrl, data.audioUrl),
          visemeUrl: resolveUrl(apiBaseUrl, data.visemeUrl),
          text: sentenceTextRef.current?.get(data.sentenceId) ?? null,
        });
      } catch (err) {
        console.error("[face] RSR speech poll failed", err);
      }
    }

    const intervalId = window.setInterval(poll, POLL_INTERVAL_MS);
    return () => {
      cancelled = true;
      window.clearInterval(intervalId);
    };
  }, [robotId, isMock, apiBaseUrl]);

  // Mock mode: press N to speak the next sample sentence.
  useEffect(() => {
    if (!isMock) return undefined;
    let mockIndex = 0;

    function onKeyDown(event: KeyboardEvent) {
      if (event.key.toLowerCase() !== MOCK_ADVANCE_KEY || playingRef.current) return;
      const sentence = MOCK_RSR_SENTENCES[mockIndex % MOCK_RSR_SENTENCES.length];
      mockIndex += 1;

      const durationMs = estimateSpeechDurationMs(sentence.text);
      const audioUrl = buildSilentWavBlobUrl(durationMs);
      const visemeUrl = buildMockVisemeTimelineBlobUrl(durationMs);
      activeUrlsRef.current = { audioUrl, visemeUrl };
      playingRef.current = true;
      setActive({
        commandId: `mock-${sentence.id}-${Date.now()}`,
        sentenceId: sentence.id,
        audioUrl,
        visemeUrl,
        text: sentence.text,
      });
    }

    window.addEventListener("keydown", onKeyDown);
    return () => window.removeEventListener("keydown", onKeyDown);
  }, [isMock]);

  const handleComplete = useCallback(() => {
    setActive(null);
    playingRef.current = false;

    if (isMock) {
      const urls = activeUrlsRef.current;
      if (urls) {
        URL.revokeObjectURL(urls.audioUrl);
        URL.revokeObjectURL(urls.visemeUrl);
      }
      activeUrlsRef.current = null;
      return;
    }

    if (!robotId) return;
    fetch(`${apiBaseUrl}/api/rsr-speech/${robotId}/pending`, { method: "DELETE" }).catch((err) => {
      console.error("[face] Failed to acknowledge RSR speech command", err);
    });
  }, [isMock, robotId, apiBaseUrl]);

  useStaticVisemePlayback(active, handleComplete);

  return { activeText: active?.text ?? null };
}
