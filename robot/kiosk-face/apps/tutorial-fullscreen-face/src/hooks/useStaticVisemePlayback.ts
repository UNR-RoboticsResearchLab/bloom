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

// Firing a fresh animateValue tween each time the active viseme changes (even
// with easeInOut and a duration spanning the whole gap) makes every segment
// independently decelerate to zero velocity right as it's replaced, then the
// next segment re-accelerates from rest — across a whole sentence that reads
// as a repeating slow-down/lurch rather than steady motion, since each pair
// of adjacent segments eases on its own clock instead of a shared one.
//
// Instead, every frame this computes a single smoothstep progress value
// between the current viseme and the next one, and drives both segments'
// weights directly off that *same* value (1-eased / eased) via setInput —
// so a segment's rise and its own later fall are always two positions on one
// continuous curve, not two independently-timed tweens that happen to meet
// at 1. Real Azure timelines (~60-150ms between events) and sparser ones
// (mock mode, ~350-500ms) both just get a shorter or longer window for that
// same curve.
const CLOSE_DURATION_SEC = 0.15;

function smoothstep(t: number): number {
  const clamped = Math.min(1, Math.max(0, t));
  return clamped * clamped * (3 - 2 * clamped);
}

// Plays a pre-recorded audio file while driving the rig's viseme weights from
// a matching pre-generated Azure viseme timeline. Unlike
// tutorial-agent-face's useVisemeMouth (which estimates visemes live from
// streaming audio with no ground truth), this just looks up the exact
// timestamped viseme for the audio's current playback position.
export function useStaticVisemePlayback(
  request: StaticPlaybackRequest,
  onComplete: () => void,
) {
  const { faceId: runtimeFaceId, setInput, animateValue } = useVizijRuntime();
  const faceId = (runtimeFaceId ?? "face").toLowerCase();
  const onCompleteRef = useRef(onComplete);
  onCompleteRef.current = onComplete;

  useEffect(() => {
    if (!request) return undefined;
    const req = request;

    let cancelled = false;
    let rafId = 0;
    let visibleSegments = new Set<FaceVisemeSegment>();
    const audio = new Audio(req.audioUrl);

    function weightPath(segment: FaceVisemeSegment): string {
      return `rig/${faceId}/visemes/${segment}.weight`;
    }

    // Snaps any segment not part of the current pair back to 0 immediately
    // (they've already finished their own curve by the time they're dropped).
    function clearStaleSegments(keep: Set<FaceVisemeSegment>) {
      for (const segment of visibleSegments) {
        if (!keep.has(segment)) {
          setInput(weightPath(segment), { float: 0 });
        }
      }
    }

    function findActiveEventIndex(events: VisemeEvent[], tMs: number): number {
      let lo = 0;
      let hi = events.length - 1;
      let result = -1;
      while (lo <= hi) {
        const mid = (lo + hi) >> 1;
        if (events[mid].timestamp_ms <= tMs) {
          result = mid;
          lo = mid + 1;
        } else {
          hi = mid - 1;
        }
      }
      return result;
    }

    function tick(events: VisemeEvent[]) {
      if (cancelled) return;
      const tMs = audio.currentTime * 1000;
      const activeIndex = findActiveEventIndex(events, tMs);
      const active = activeIndex >= 0 ? events[activeIndex] : null;
      const next = activeIndex >= 0 ? events[activeIndex + 1] : undefined;

      const activeSegment = active ? mapAzureViseme(active.viseme_id).segment : null;
      const nextSegment = next ? mapAzureViseme(next.viseme_id).segment : null;

      let progress = 1;
      if (active && next) {
        const span = next.timestamp_ms - active.timestamp_ms;
        progress = span > 0 ? (tMs - active.timestamp_ms) / span : 1;
      }
      const eased = smoothstep(progress);

      const keep = new Set<FaceVisemeSegment>();
      if (activeSegment) keep.add(activeSegment);
      if (nextSegment) keep.add(nextSegment);
      clearStaleSegments(keep);

      if (activeSegment && activeSegment === nextSegment) {
        // Consecutive events map to the same segment (no real transition
        // yet) — hold it fully open rather than tracking `eased`, which
        // measures progress toward whatever comes *after* nextSegment.
        setInput(weightPath(activeSegment), { float: 1 });
      } else {
        if (activeSegment) setInput(weightPath(activeSegment), { float: 1 - eased });
        if (nextSegment) setInput(weightPath(nextSegment), { float: eased });
      }
      visibleSegments = keep;

      if (!audio.ended) {
        rafId = requestAnimationFrame(() => tick(events));
      }
    }

    function closeSegments() {
      for (const segment of visibleSegments) {
        animateValue(weightPath(segment), { float: 0 }, { duration: CLOSE_DURATION_SEC, easing: "easeOut" });
      }
      visibleSegments = new Set();
    }

    function finish() {
      closeSegments();
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
      closeSegments();
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [request, faceId, setInput, animateValue]);
}
