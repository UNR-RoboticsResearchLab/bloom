import { useCallback, useRef } from "react";
import { useVizijRuntime, type PoseRigConfig } from "@vizij/runtime-react";
import { createPoseNameMap } from "./posePaths";

const EMOTION_TRANSITION_SEC = 0.4;

// bloom_face/face_node.py's 8 pygame emotions -> the rig's 7 authored
// "Emotions" group poses (Neutral, Surprise, Sleepy, Angry, Concerned, Sad,
// Happy). Not a 1:1 mapping -- excited/calm/thinking have no rig equivalent,
// so they fall back to the closest available pose.
const EMOTION_TO_POSE_NAME: Record<string, string> = {
  neutral: "neutral",
  happy: "happy",
  sad: "sad",
  excited: "happy",
  calm: "neutral",
  surprised: "surprise",
  thinking: "concerned",
  sleepy: "sleepy",
};

// Drives the rig's emotion pose weights from ROS-bridge emotion events, and
// lets viseme playback (useLiveVisemePlayback) temporarily suppress the
// current emotion pose while a viseme pose is actively driving the same
// underlying mouth channels -- unconfirmed whether the rig's pose-driver
// blends overlapping pose weights gracefully, so this avoids relying on it.
export function useFaceEmotion(poseConfig: PoseRigConfig | null) {
  const { faceId: runtimeFaceId, animateValue, setInput } = useVizijRuntime();
  const faceId = (runtimeFaceId ?? "face").toLowerCase();
  const currentPathRef = useRef<string | null>(null);
  const suppressedRef = useRef(false);

  const setEmotion = useCallback(
    (rawEmotion: string) => {
      if (!poseConfig) return;
      const nameMap = createPoseNameMap(faceId, poseConfig);
      const poseName = EMOTION_TO_POSE_NAME[rawEmotion.toLowerCase()] ?? rawEmotion.toLowerCase();
      const binding = nameMap.get(poseName);
      if (!binding) {
        console.warn(`[face] No rig pose for emotion "${rawEmotion}" (looked for "${poseName}")`);
        return;
      }

      const previousPath = currentPathRef.current;
      currentPathRef.current = binding.path;

      if (previousPath && previousPath !== binding.path) {
        animateValue(previousPath, { float: 0 }, { duration: EMOTION_TRANSITION_SEC });
      }
      if (!suppressedRef.current) {
        animateValue(binding.path, { float: 1 }, { duration: EMOTION_TRANSITION_SEC });
      }
    },
    [poseConfig, faceId, animateValue],
  );

  // Called by viseme playback when it starts/stops driving the mouth, so the
  // active emotion pose's mouth-shape contribution doesn't fight the viseme.
  const suppressForViseme = useCallback(
    (suppressed: boolean) => {
      suppressedRef.current = suppressed;
      const path = currentPathRef.current;
      if (!path) return;
      if (suppressed) {
        setInput(path, { float: 0 });
      } else {
        animateValue(path, { float: 1 }, { duration: EMOTION_TRANSITION_SEC });
      }
    },
    [setInput, animateValue],
  );

  return { setEmotion, suppressForViseme };
}
