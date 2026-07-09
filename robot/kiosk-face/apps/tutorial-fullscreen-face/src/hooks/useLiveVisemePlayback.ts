import { useCallback, useRef } from "react";
import { useVizijRuntime } from "@vizij/runtime-react";
import { mapAzureViseme, type FaceVisemeSegment } from "../azureVisemeMapping";

export type LiveVisemeEvent = { timestamp_ms: number; viseme_id: number };

const CLOSE_DURATION_SEC = 0.15;

function smoothstep(t: number): number {
  const clamped = Math.min(1, Math.max(0, t));
  return clamped * clamped * (3 - 2 * clamped);
}

function findActiveEventIndex(events: LiveVisemeEvent[], tMs: number): number {
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

// Drives the same viseme-pose crossfade as useStaticVisemePlayback, but
// timed against wall-clock elapsed time since a start_audio_sync event
// rather than an <audio> element's currentTime -- audio for live (ROS
// bridge-driven) speech plays through the robot's own speakers via
// tts_node.py/rsr_speech_node.py's pygame.mixer, not through the browser, so
// there's no local <audio> element whose playback position could serve as
// the clock. This mirrors bloom_face's own BlossomFace.update(), which
// advances its viseme index off (time.time() - audio_start_time) the same way.
export function useLiveVisemePlayback(onModeChange: (active: boolean) => void) {
  const { faceId: runtimeFaceId, setInput, animateValue } = useVizijRuntime();
  const faceId = (runtimeFaceId ?? "face").toLowerCase();

  const eventsRef = useRef<LiveVisemeEvent[]>([]);
  const rafIdRef = useRef(0);
  const startTimeRef = useRef<number | null>(null);
  const visibleSegmentsRef = useRef<Set<FaceVisemeSegment>>(new Set());

  const weightPath = useCallback(
    (segment: FaceVisemeSegment) => `rig/${faceId}/visemes/${segment}.weight`,
    [faceId],
  );

  const clearStaleSegments = useCallback(
    (keep: Set<FaceVisemeSegment>) => {
      for (const segment of visibleSegmentsRef.current) {
        if (!keep.has(segment)) {
          setInput(weightPath(segment), { float: 0 });
        }
      }
    },
    [setInput, weightPath],
  );

  const tick = useCallback(() => {
    const startTime = startTimeRef.current;
    if (startTime === null) return;

    const tMs = performance.now() - startTime;
    const events = eventsRef.current;
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
      setInput(weightPath(activeSegment), { float: 1 });
    } else {
      if (activeSegment) setInput(weightPath(activeSegment), { float: 1 - eased });
      if (nextSegment) setInput(weightPath(nextSegment), { float: eased });
    }
    visibleSegmentsRef.current = keep;

    rafIdRef.current = requestAnimationFrame(tick);
  }, [clearStaleSegments, setInput, weightPath]);

  const setTimeline = useCallback((events: LiveVisemeEvent[]) => {
    eventsRef.current = [...events].sort((a, b) => a.timestamp_ms - b.timestamp_ms);
  }, []);

  const start = useCallback(() => {
    startTimeRef.current = performance.now();
    onModeChange(true);
    cancelAnimationFrame(rafIdRef.current);
    rafIdRef.current = requestAnimationFrame(tick);
  }, [onModeChange, tick]);

  const stop = useCallback(() => {
    cancelAnimationFrame(rafIdRef.current);
    startTimeRef.current = null;
    for (const segment of visibleSegmentsRef.current) {
      animateValue(weightPath(segment), { float: 0 }, { duration: CLOSE_DURATION_SEC, easing: "easeOut" });
    }
    visibleSegmentsRef.current = new Set();
    onModeChange(false);
  }, [animateValue, onModeChange, weightPath]);

  return { setTimeline, start, stop };
}
