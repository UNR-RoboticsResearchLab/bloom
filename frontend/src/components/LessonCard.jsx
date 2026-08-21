// LessonCard.jsx
// Canonical lesson card — used by the Browse Lessons page and every
// dashboard that lists lessons, so they render identically. Built on the
// shared Card/Badge primitives (components/ui) rather than bespoke markup.

import React from "react";
import Card from "./ui/Card";
import Badge from "./ui/Badge";
import { getLessonTypeConfig } from "../utils/lessonTypes";

export default function LessonCard({ lesson, onClick }) {
  const typeKey = lesson?.lessonType ?? lesson?.LessonType ?? 0;
  const config = getLessonTypeConfig(typeKey);
  const title = lesson?.title ?? lesson?.Title ?? "Untitled";
  const description = lesson?.description ?? lesson?.Description ?? "";
  const stepCount = lesson?.totalSteps ?? lesson?.TotalSteps ?? 0;
  const objectives = lesson?.learningObjectives ?? lesson?.LearningObjectives ?? [];
  const createdDate = lesson?.createdDate ?? lesson?.CreatedDate;
  const createdByName = lesson?.createdByName ?? lesson?.CreatedByName;
  const isPublic = lesson?.isPublic ?? lesson?.IsPublic ?? true;
  const hasObjectives = Array.isArray(objectives) && objectives.length > 0;

  return (
    <Card
      as="button"
      interactive
      accentClassName={config.border}
      onClick={onClick}
      aria-label={`View lesson: ${title}`}
    >
      <div className="flex items-start justify-between gap-4">
        <div className="flex-1 min-w-0">
          <p className="text-base font-semibold text-gray-900 group-hover:text-indigo-600 transition-colors">
            {title}
          </p>
          {description && <p className="mt-1 text-sm text-gray-500 line-clamp-2">{description}</p>}
        </div>
        <div className="flex shrink-0 items-center gap-2">
          {!isPublic && <Badge color="bg-gray-100 text-gray-600">Private</Badge>}
          <Badge color={config.badge}>{config.label}</Badge>
        </div>
      </div>

      {hasObjectives && (
        <ul className="mt-3 space-y-1" aria-label="Learning objectives">
          {objectives.map((obj, i) => (
            <li key={i} className="flex items-start gap-2 text-xs text-gray-600">
              <span
                className="mt-0.5 flex-shrink-0 w-4 h-4 rounded-full bg-green-100 text-green-700 flex items-center justify-center text-xs font-bold"
                aria-hidden="true"
              >
                ✓
              </span>
              <span>{obj}</span>
            </li>
          ))}
        </ul>
      )}

      <div className="mt-3 flex items-center gap-4 text-xs text-gray-400">
        {stepCount > 0 && <span>{stepCount} steps</span>}
        {hasObjectives && (
          <span>
            {objectives.length} objective{objectives.length !== 1 ? "s" : ""}
          </span>
        )}
        {createdByName && <span className={createdDate ? "" : "ml-auto"}>by {createdByName}</span>}
        {createdDate && <span className="ml-auto">{new Date(createdDate).toLocaleDateString()}</span>}
      </div>
    </Card>
  );
}
