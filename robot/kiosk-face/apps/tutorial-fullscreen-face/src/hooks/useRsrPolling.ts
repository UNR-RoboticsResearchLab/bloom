import { useCallback, useEffect, useRef, useState } from "react";
import { useStaticVisemePlayback } from "./useStaticVisemePlayback";

const POLL_INTERVAL_MS = 2000;

type PendingSpeechResponse = {
  commandId: string;
  sentenceId: number;
  audioUrl: string;
  visemeUrl: string;
};

function resolveUrl(base: string, path: string): string {
  if (/^https?:\/\//i.test(path)) return path;
  return `${base.replace(/\/$/, "")}${path}`;
}

// Polls the bloom backend for a queued RSR sentence to speak. The kiosk
// browser is opened with ?robotId=<id> baked into the launch URL; if that's
// absent this hook is a no-op (so the plain hotkey-demo behavior of this app
// is unaffected when not used for RSR administration).
export function useRsrPolling() {
  const robotIdRef = useRef<string | null>(null);
  if (robotIdRef.current === null) {
    robotIdRef.current = new URLSearchParams(window.location.search).get("robotId");
  }
  const apiBaseUrl = (import.meta.env.VITE_BLOOM_API_URL as string | undefined) ?? "http://localhost:5000";

  const [active, setActive] = useState<PendingSpeechResponse | null>(null);
  const playingRef = useRef(false);

  useEffect(() => {
    const robotId = robotIdRef.current;
    if (!robotId) return undefined;
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
  }, [apiBaseUrl]);

  const handleComplete = useCallback(() => {
    const robotId = robotIdRef.current;
    setActive(null);
    playingRef.current = false;
    if (!robotId) return;
    fetch(`${apiBaseUrl}/api/rsr-speech/${robotId}/pending`, { method: "DELETE" }).catch((err) => {
      console.error("[face] Failed to acknowledge RSR speech command", err);
    });
  }, [apiBaseUrl]);

  useStaticVisemePlayback(active, handleComplete);
}
