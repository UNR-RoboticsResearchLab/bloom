// LessonPreview.jsx
// Read-only rendering of a lesson as currently drafted in LessonBuilder --
// lets an author see how the lesson will present before saving. Renders
// directly from the builder's local (possibly unsaved, possibly invalid)
// state, so every field is treated as optional.

import React, { useState } from "react";

const TYPE_LABELS = { 0: "Language", 1: "Speech Therapy" };

const BEHAVIOR_LABELS = {
  behavior: "Behavior",
  facial_expression: "Facial Expression",
  gaze: "Gaze",
  head_movement: "Head Movement",
};

function parseInteraction(interaction) {
  if (!interaction) return null;
  if (typeof interaction === "object") return interaction;
  try {
    return JSON.parse(interaction);
  } catch {
    return null;
  }
}

function humanizeKey(key) {
  return key.replace(/_/g, " ").replace(/\b\w/g, (c) => c.toUpperCase());
}

function PreviewStep({ step, index }) {
  const [open, setOpen] = useState(index === 0);
  const interaction = parseInteraction(step.interaction);
  const behaviorEntries = Object.entries(step.behaviors || {}).filter(([, v]) => v);
  const interactionEntries = Object.entries(interaction || {}).filter(
    ([, v]) => v !== null && v !== undefined && v !== ""
  );

  return (
    <li className="rounded-lg border border-gray-100 overflow-hidden">
      <button
        type="button"
        onClick={() => setOpen((o) => !o)}
        aria-expanded={open}
        className="w-full flex items-center gap-3 bg-gray-50 p-3 hover:bg-gray-100 transition-colors text-left"
      >
        <span
          className="flex-shrink-0 w-7 h-7 rounded-full bg-indigo-100 text-indigo-700 flex items-center justify-center text-xs font-bold"
          aria-label={`Step ${index + 1}`}
        >
          {index + 1}
        </span>
        <span className="flex-1 text-sm font-medium text-gray-800">
          {step.title || `Step ${index + 1}`}
        </span>
        {step.type && (
          <span className="text-xs px-2 py-0.5 rounded-full bg-gray-200 text-gray-600 capitalize">
            {step.type.replace(/_/g, " ")}
          </span>
        )}
        <span className="text-gray-400 text-xs ml-1">{open ? "▲" : "▼"}</span>
      </button>

      {open && (
        <div className="border-t border-gray-100 bg-white px-4 py-3 space-y-4">
          {step.script ? (
            <div>
              <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">
                Script
              </dt>
              <dd className="mt-0.5 text-sm text-gray-800 whitespace-pre-wrap break-words">
                {step.script}
              </dd>
            </div>
          ) : (
            <p className="text-sm text-gray-400 italic">No script written yet.</p>
          )}

          {step.timingSeconds != null && (
            <div>
              <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">
                Timing
              </dt>
              <dd className="mt-0.5 text-sm text-gray-800">{step.timingSeconds}s</dd>
            </div>
          )}

          {behaviorEntries.length > 0 && (
            <dl className="grid grid-cols-1 sm:grid-cols-2 gap-x-6 gap-y-2">
              {behaviorEntries.map(([key, value]) => (
                <div key={key}>
                  <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">
                    {BEHAVIOR_LABELS[key] || humanizeKey(key)}
                  </dt>
                  <dd className="mt-0.5 text-sm text-gray-800">{value}</dd>
                </div>
              ))}
            </dl>
          )}

          {interactionEntries.length > 0 && (
            <div>
              <p className="text-xs font-semibold text-gray-400 uppercase tracking-wide mb-2">
                Interaction
              </p>
              <dl className="grid grid-cols-1 sm:grid-cols-2 gap-x-6 gap-y-2 rounded-lg bg-gray-50 p-3">
                {interactionEntries.map(([key, value]) => (
                  <div key={key}>
                    <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">
                      {humanizeKey(key)}
                    </dt>
                    <dd className="mt-0.5 text-sm text-gray-800 whitespace-pre-wrap break-words">
                      {String(value)}
                    </dd>
                  </div>
                ))}
              </dl>
            </div>
          )}
        </div>
      )}
    </li>
  );
}

export default function LessonPreview({ lesson, onClose }) {
  const title = lesson.title?.trim() || "Untitled Lesson";
  const objectives = (lesson.learningObjectives || []).filter((o) => o?.trim());
  const steps = lesson.steps || [];

  return (
    <div
      className="fixed inset-0 z-[60] flex items-center justify-center bg-black/40 p-4 sm:p-8"
      role="dialog"
      aria-modal="true"
      aria-label="Lesson preview"
    >
      <div className="mx-auto flex max-h-[90vh] w-full max-w-3xl flex-col overflow-hidden rounded-2xl bg-gray-50 shadow-xl">
        <div className="flex items-center justify-between border-b border-gray-200 bg-white px-6 py-4">
          <h2 className="text-lg font-semibold text-gray-900">Preview</h2>
          <button
            type="button"
            onClick={onClose}
            className="text-gray-400 hover:text-gray-600 text-xl leading-none"
            aria-label="Close preview"
          >
            &times;
          </button>
        </div>

        <div className="flex-1 overflow-y-auto space-y-6 p-6">
          <header className="rounded-2xl bg-blue-50 border border-gray-200 p-6 shadow-sm">
            <span className="inline-flex items-center rounded-full bg-blue-100 px-3 py-1 text-xs font-semibold text-blue-700">
              {TYPE_LABELS[lesson.lessonType] ?? "Language"}
            </span>
            <h1 className="mt-3 text-2xl font-bold text-gray-900 leading-tight">{title}</h1>
            {lesson.description && (
              <p className="mt-2 text-sm text-gray-600 leading-relaxed">{lesson.description}</p>
            )}
            <div className="mt-4 flex flex-wrap gap-4 text-sm text-gray-500">
              <span>
                <strong className="text-gray-800">{steps.length}</strong> step
                {steps.length !== 1 ? "s" : ""}
              </span>
              {objectives.length > 0 && (
                <span>
                  <strong className="text-gray-800">{objectives.length}</strong> learning
                  objective{objectives.length !== 1 ? "s" : ""}
                </span>
              )}
            </div>
          </header>

          {objectives.length > 0 && (
            <section className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
              <h2 className="text-base font-semibold text-gray-900">Learning Objectives</h2>
              <ul className="mt-3 space-y-2" role="list">
                {objectives.map((obj, i) => (
                  <li key={i} className="flex items-start gap-3 text-sm text-gray-700">
                    <span
                      className="mt-0.5 flex-shrink-0 w-5 h-5 rounded-full bg-green-100 text-green-700 flex items-center justify-center text-xs font-bold"
                      aria-hidden="true"
                    >
                      ✓
                    </span>
                    <span>{obj}</span>
                  </li>
                ))}
              </ul>
            </section>
          )}

          <section className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
            <h2 className="text-base font-semibold text-gray-900">
              Steps <span className="ml-1 text-sm font-normal text-gray-400">({steps.length})</span>
            </h2>
            {steps.length === 0 ? (
              <p className="mt-3 text-sm text-gray-400 italic">No steps yet.</p>
            ) : (
              <ol className="mt-3 space-y-2" role="list">
                {steps.map((step, i) => (
                  <PreviewStep key={step.id ?? i} step={step} index={i} />
                ))}
              </ol>
            )}
          </section>
        </div>

        <div className="flex items-center justify-end gap-3 border-t border-gray-200 bg-white px-6 py-4">
          <button
            type="button"
            onClick={onClose}
            className="rounded-md px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-100"
          >
            Close Preview
          </button>
        </div>
      </div>
    </div>
  );
}
