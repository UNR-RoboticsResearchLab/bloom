import type { PoseRigConfig, PoseDefinition } from "@vizij/runtime-react";

// Shared between usePoseHotkeys (keyboard demo) and useFaceEmotion (ROS
// bridge-driven emotion switching) so both derive weight paths the same way.

export function slugify(value: string): string {
  return value
    .toLowerCase()
    .trim()
    .replace(/[^a-z0-9]+/g, "_")
    .replace(/^_+|_+$/g, "");
}

export function normalizeGroup(group?: string | null): string {
  const normalized = slugify(group ?? "poses");
  if (!normalized) {
    return "poses";
  }
  if (normalized.startsWith("emotion")) {
    return "emotions";
  }
  if (normalized.startsWith("viseme")) {
    return "visemes";
  }
  return normalized;
}

export type PosePathBinding = {
  pose: PoseDefinition;
  path: string;
};

// pose.id -> full rig weight path (e.g. rig/hugo/emotions/happy.weight)
export function createPosePathMap(
  faceId: string,
  poseConfig: PoseRigConfig,
): Map<string, string> {
  const map = new Map<string, string>();
  const counters = new Map<string, Map<string, number>>();
  const poses = poseConfig.poses ?? [];

  poses.forEach((pose, index) => {
    const group = normalizeGroup(pose.group);
    let slug = slugify(pose.name ?? "") || slugify(pose.id ?? "");
    if (!slug) {
      slug = `pose_${index + 1}`;
    }

    const groupCounts = counters.get(group) ?? new Map<string, number>();
    const seen = groupCounts.get(slug) ?? 0;
    groupCounts.set(slug, seen + 1);
    counters.set(group, groupCounts);
    const uniqueSlug = seen === 0 ? slug : `${slug}_${seen + 1}`;

    map.set(pose.id, `rig/${faceId}/${group}/${uniqueSlug}.weight`);
  });

  return map;
}

// lowercased pose.name -> { pose, path }, for looking a pose up by name
// (e.g. an incoming "happy" emotion string) rather than by hotkey position.
export function createPoseNameMap(
  faceId: string,
  poseConfig: PoseRigConfig,
): Map<string, PosePathBinding> {
  const pathMap = createPosePathMap(faceId, poseConfig);
  const nameMap = new Map<string, PosePathBinding>();
  for (const pose of poseConfig.poses ?? []) {
    const path = pathMap.get(pose.id);
    const name = pose.name?.toLowerCase();
    if (path && name && !nameMap.has(name)) {
      nameMap.set(name, { pose, path });
    }
  }
  return nameMap;
}
