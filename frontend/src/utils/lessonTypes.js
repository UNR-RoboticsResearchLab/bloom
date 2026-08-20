// lessonTypes.js
// Single source of truth for lesson-type display config (label, badge color,
// card accent color), shared by every place that renders a lesson card.

export const LESSON_TYPE_CONFIG = {
  0: {
    label: "Language",
    bg: "bg-blue-50",
    badge: "bg-blue-100 text-blue-700",
    border: "border-l-blue-400",
    accent: "bg-blue-500",
  },
  1: {
    label: "Speech Therapy",
    bg: "bg-violet-50",
    badge: "bg-violet-100 text-violet-700",
    border: "border-l-violet-400",
    accent: "bg-violet-500",
  },
};

export function getLessonTypeConfig(typeKey) {
  return LESSON_TYPE_CONFIG[typeKey] ?? LESSON_TYPE_CONFIG[0];
}
