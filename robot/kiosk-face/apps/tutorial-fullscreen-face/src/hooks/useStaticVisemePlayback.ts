import { useEffect, useRef } from "react";
import { useVizijRuntime } from "@vizij/runtime-react";
import { mapAzureViseme, type FaceVisemeSegment } from "../azureVisemeMapping";

export type VisemeEvent = {
  timestamp_ms: number;
  viseme_id: number;
};

export type StaticPlaybackRequest = {
  audioUrl: string;
  visemeUrl: string;
} | null;

const CROSSFADE_SEC = 0.08;

// Plays a pre-recorded audio file while driving the rig's viseme weights from
// a matching pre-generated Azure viseme timeline. Unlike
// tutorial-agent-face's useVisemeMouth (which estimates visemes live from
// streaming audio with no ground truth), this just looks up the exact
// timestamped viseme for the audio's current playback position.
export function useStaticVisemePlayback(
  request: StaticPlaybackRequest,
  onComplete: () => void,
) {
  const { faceId: runtimeFaceId, animateValue } = useVizijRuntime();
  const faceId = (runtimeFaceId ?? "face").toLowerCase();
  const onCompleteRef = useRef(onComplete);
  onCompleteRef.current = onComplete;

  useEffect(() => {
    if (!request) return undefined;
    const req = request;

    let cancelled = false;
    let rafId = 0;
    let activeSegment: FaceVisemeSegment | null = null;
    const audio = new Audio(req.audioUrl);

    function setSegment(segment: FaceVisemeSegment | null) {
      if (segment === activeSegment) return;
      if (activeSegment) {
        animateValue(`rig/${faceId}/visemes/${activeSegment}.weight`, { float: 0 }, { duration: CROSSFADE_SEC });
      }
      if (segment) {
        animateValue(`rig/${faceId}/visemes/${segment}.weight`, { float: 1 }, { duration: CROSSFADE_SEC });
      }
      activeSegment = segment;
    }

    function findActiveEvent(events: VisemeEvent[], tMs: number): VisemeEvent | null {
      let lo = 0;
      let hi = events.length - 1;
      let result: VisemeEvent | null = null;
      while (lo <= hi) {
        const mid = (lo + hi) >> 1;
        if (events[mid].timestamp_ms <= tMs) {
          result = events[mid];
          lo = mid + 1;
        } else {
          hi = mid - 1;
        }
      }
      return result;
    }

    function tick(events: VisemeEvent[]) {
      if (cancelled) return;
      const active = findActiveEvent(events, audio.currentTime * 1000);
      setSegment(active ? mapAzureViseme(active.viseme_id).segment : null);
      if (!audio.ended) {
        rafId = requestAnimationFrame(() => tick(events));
      }
    }

    function finish() {
      setSegment(null);
      if (!cancelled) onCompleteRef.current();
    }

    async function start() {
      let events: VisemeEvent[] = [];
      try {
        const res = await fetch(req.visemeUrl);
        events = (await res.json()) as VisemeEvent[];
        events = [...events].sort((a, b) => a.timestamp_ms - b.timestamp_ms);
      } catch (err) {
        console.error("[face] Failed to load viseme timeline", err);
      }
      if (cancelled) return;

      audio.addEventListener("ended", finish);
      audio.addEventListener("error", finish);

      try {
        await audio.play();
      } catch (err) {
        console.error("[face] Failed to play sentence audio", err);
        finish();
        return;
      }
      if (cancelled) return;
      rafId = requestAnimationFrame(() => tick(events));
    }

    start();

    return () => {
      cancelled = true;
      cancelAnimationFrame(rafId);
      audio.pause();
      audio.removeEventListener("ended", finish);
      audio.removeEventListener("error", finish);
      setSegment(null);
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [request, faceId, animateValue]);
}
