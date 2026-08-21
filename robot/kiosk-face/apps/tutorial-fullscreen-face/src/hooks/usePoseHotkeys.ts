import { useEffect, useMemo } from "react";
import { useVizijRuntime, type PoseRigConfig } from "@vizij/runtime-react";
import { createPosePathMap, type PosePathBinding } from "./posePaths";

export const POSE_HOTKEY_ORDER = [
  "Digit1",
  "Digit2",
  "Digit3",
  "Digit4",
  "Digit5",
] as const;

export function usePoseHotkeys(
  poseConfig: PoseRigConfig | null,
  enabled: boolean,
) {
  const { faceId: runtimeFaceId, animateValue } = useVizijRuntime();
  const faceId = (runtimeFaceId ?? "face").toLowerCase();
  const posePathMap = useMemo(() => {
    if (!poseConfig) {
      return null;
    }
    return createPosePathMap(faceId, poseConfig);
  }, [faceId, poseConfig]);

  useEffect(() => {
    if (!enabled || !poseConfig || !posePathMap || posePathMap.size === 0) {
      return;
    }

    const poses = poseConfig.poses ?? [];

    const bindings = POSE_HOTKEY_ORDER.reduce<Map<string, PosePathBinding>>(
      (acc, code, index) => {
        const pose = poses[index];
        if (pose) {
          const path = posePathMap.get(pose.id);
          if (path) {
            acc.set(code, { pose, path });
          } else {
            console.warn(
              `[fullscreen-face] Missing pose path for ${pose.name ?? pose.id}`,
            );
          }
        }
        return acc;
      },
      new Map(),
    );

    if (bindings.size === 0) {
      return;
    }

    const activeKeys = new Set<string>();

    const applyWeight = (binding: PosePathBinding, weight: number) => {
      animateValue(binding.path, { float: weight }, { duration: 2 });
    };

    const handleKeyDown = (event: globalThis.KeyboardEvent) => {
      const binding = bindings.get(event.code);
      if (!binding) {
        return;
      }
      if (activeKeys.has(event.code)) {
        return;
      }
      activeKeys.add(event.code);
      applyWeight(binding, 1);
    };

    const handleKeyUp = (event: globalThis.KeyboardEvent) => {
      const binding = bindings.get(event.code);
      if (!binding) {
        return;
      }
      activeKeys.delete(event.code);
      applyWeight(binding, 0);
    };

    window.addEventListener("keydown", handleKeyDown);
    window.addEventListener("keyup", handleKeyUp);

    return () => {
      window.removeEventListener("keydown", handleKeyDown);
      window.removeEventListener("keyup", handleKeyUp);
      bindings.forEach((binding) => applyWeight(binding, 0));
    };
  }, [animateValue, enabled, faceId, poseConfig, posePathMap]);
}
