// LessonStepBuilder.jsx
// Collapsible card for editing a single LessonStep within the LessonBuilder.
// Calls onChange(stepDto) whenever the step state changes.

import React, { useState } from "react";

export const STEP_TYPES = [
  "instruction",
  "introduction",
  "question",
  "wait_for_response",
  "feedback",
  "transition",
  "conclusion",
];

const BEHAVIOR_FIELDS = [
  { key: "behavior", label: "Behavior" },
  { key: "facial_expression", label: "Facial Expression" },
  { key: "gaze", label: "Gaze" },
  { key: "head_movement", label: "Head Movement" },
  { key: "posture", label: "Posture" },
];

// Seed interaction fields when a type is selected
const INTERACTION_TEMPLATES = {
  wait_for_response: [
    { key: "wait_for_response", value: "true" },
    { key: "timeout_seconds", value: "10" },
  ],
  question: [
    { key: "wait_for_response", value: "true" },
    { key: "correct_answer", value: "" },
    { key: "repeat_count", value: "3" },
  ],
  feedback: [
    { key: "feedback_type", value: "" },
    { key: "message", value: "" },
  ],
  transition: [{ key: "delay_seconds", value: "0" }],
};

function parseBehaviors(jsonStr) {
  const defaults = { behavior: "", facial_expression: "", gaze: "", head_movement: "", posture: "" };
  if (!jsonStr) return defaults;
  try {
    return { ...defaults, ...JSON.parse(jsonStr) };
  } catch {
    return defaults;
  }
}

function parseInteraction(jsonStr) {
  if (!jsonStr) return [];
  try {
    return Object.entries(JSON.parse(jsonStr)).map(([key, value]) => ({
      key,
      value: String(value),
    }));
  } catch {
    return [];
  }
}

export function buildStepDto(stepOrder, core, behaviors, interactionFields, existingId) {
  const behaviorsObj = Object.fromEntries(
    Object.entries(behaviors).filter(([, v]) => v.trim() !== "")
  );

  const interactionObj = {};
  for (const { key, value } of interactionFields) {
    if (!key.trim()) continue;
    if (value === "true") interactionObj[key] = true;
    else if (value === "false") interactionObj[key] = false;
    else if (value !== "" && !isNaN(Number(value))) interactionObj[key] = Number(value);
    else interactionObj[key] = value;
  }

  return {
    ...(existingId ? { id: existingId } : {}),
    stepOrder,
    type: core.type,
    script: core.script,
    timingSeconds: core.timingSeconds !== "" ? Number(core.timingSeconds) : null,
    visualAid: core.visualAid || null,
    behaviors: Object.keys(behaviorsObj).length > 0 ? JSON.stringify(behaviorsObj) : null,
    interaction: Object.keys(interactionObj).length > 0 ? JSON.stringify(interactionObj) : null,
  };
}

export default function LessonStepBuilder({ step, stepNumber, onChange, onRemove, onMoveUp, onMoveDown, isFirst, isLast }) {
  const [expanded, setExpanded] = useState(!step.type);

  const [core, setCore] = useState({
    type: step.type ?? "",
    script: step.script ?? "",
    timingSeconds: step.timingSeconds ?? "",
    visualAid: step.visualAid ?? "",
  });

  const [behaviors, setBehaviors] = useState(() => parseBehaviors(step.behaviors));
  const [interactionFields, setInteractionFields] = useState(() => parseInteraction(step.interaction));

  function emit(nextCore, nextBehaviors, nextInteraction) {
    const dto = buildStepDto(stepNumber, nextCore, nextBehaviors, nextInteraction, step.id);
    onChange(dto);
  }

  function handleCoreChange(field, value) {
    const next = { ...core, [field]: value };
    setCore(next);

    if (field === "type" && INTERACTION_TEMPLATES[value]) {
      const seeded = INTERACTION_TEMPLATES[value];
      setInteractionFields(seeded);
      emit(next, behaviors, seeded);
    } else {
      emit(next, behaviors, interactionFields);
    }
  }

  function handleBehaviorChange(key, value) {
    const next = { ...behaviors, [key]: value };
    setBehaviors(next);
    emit(core, next, interactionFields);
  }

  function addInteractionField() {
    const next = [...interactionFields, { key: "", value: "" }];
    setInteractionFields(next);
    emit(core, behaviors, next);
  }

  function removeInteractionField(index) {
    const next = interactionFields.filter((_, i) => i !== index);
    setInteractionFields(next);
    emit(core, behaviors, next);
  }

  function updateInteractionField(index, part, value) {
    const next = interactionFields.map((f, i) => (i === index ? { ...f, [part]: value } : f));
    setInteractionFields(next);
    emit(core, behaviors, next);
  }

  const inputClass =
    "block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm";
  const labelClass = "block text-sm font-medium text-gray-700";

  const summary = core.type
    ? `${core.type}${core.script ? ` — ${core.script.slice(0, 60)}${core.script.length > 60 ? "…" : ""}` : ""}`
    : "Untitled step";

  return (
    <div className="overflow-hidden rounded-2xl border border-gray-100 bg-white shadow-sm">
      {/* Card header */}
      <div className="flex items-center gap-3 bg-gray-50/70 px-5 py-4">
        <span className="flex h-8 w-8 shrink-0 items-center justify-center rounded-full bg-indigo-100 text-sm font-semibold text-indigo-700">
          {stepNumber}
        </span>

        <button
          type="button"
          className="flex-1 text-left text-sm font-medium text-gray-800 hover:text-indigo-600 truncate"
          onClick={() => setExpanded((v) => !v)}
        >
          {summary}
        </button>

        <div className="flex items-center gap-1 shrink-0">
          <button
            type="button"
            onClick={onMoveUp}
            disabled={isFirst}
            title="Move up"
            className="rounded p-1 text-gray-400 hover:bg-gray-100 disabled:opacity-30"
          >
            ↑
          </button>
          <button
            type="button"
            onClick={onMoveDown}
            disabled={isLast}
            title="Move down"
            className="rounded p-1 text-gray-400 hover:bg-gray-100 disabled:opacity-30"
          >
            ↓
          </button>
          <button
            type="button"
            onClick={onRemove}
            title="Remove step"
            className="rounded p-1 text-red-400 hover:bg-red-50"
          >
            ✕
          </button>
          <button
            type="button"
            onClick={() => setExpanded((v) => !v)}
            className="rounded p-1 text-gray-400 hover:bg-gray-100"
          >
            {expanded ? "▲" : "▼"}
          </button>
        </div>
      </div>

      {/* Expanded body */}
      {expanded && (
        <div className="border-t border-gray-100 px-5 pb-5 pt-5 space-y-5">
          {/* Type + Timing */}
          <div className="grid gap-4 sm:grid-cols-2">
            <div>
              <label className={labelClass}>
                Type <span className="text-red-500">*</span>
              </label>
              <div className="mt-1">
                <select
                  value={core.type}
                  onChange={(e) => handleCoreChange("type", e.target.value)}
                  className={inputClass}
                >
                  <option value="">Select type</option>
                  {STEP_TYPES.map((t) => (
                    <option key={t} value={t}>
                      {t}
                    </option>
                  ))}
                </select>
              </div>
            </div>

            <div>
              <label className={labelClass}>Timing (seconds)</label>
              <div className="mt-1">
                <input
                  type="number"
                  min={0}
                  value={core.timingSeconds}
                  onChange={(e) => handleCoreChange("timingSeconds", e.target.value)}
                  placeholder="Optional"
                  className={inputClass}
                />
              </div>
            </div>
          </div>

          {/* Script */}
          <div>
            <label className={labelClass}>
              Script <span className="text-red-500">*</span>
            </label>
            <div className="mt-1">
              <textarea
                rows={3}
                value={core.script}
                onChange={(e) => handleCoreChange("script", e.target.value)}
                placeholder="What the robot will say or do..."
                className="block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
              />
            </div>
          </div>

          {/* Visual Aid */}
          <div>
            <label className={labelClass}>Visual Aid</label>
            <div className="mt-1">
              <input
                type="text"
                value={core.visualAid}
                onChange={(e) => handleCoreChange("visualAid", e.target.value)}
                placeholder='filename.png or ["a.png","b.png"]'
                className={inputClass}
              />
            </div>
          </div>

          {/* Behaviors */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">
              Behaviors
            </legend>
            <div className="mt-2 grid gap-3 sm:grid-cols-2 lg:grid-cols-3">
              {BEHAVIOR_FIELDS.map(({ key, label }) => (
                <div key={key}>
                  <label className="block text-xs font-medium text-gray-600">{label}</label>
                  <input
                    type="text"
                    value={behaviors[key]}
                    onChange={(e) => handleBehaviorChange(key, e.target.value)}
                    placeholder="Optional"
                    className="mt-1 block w-full rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
                  />
                </div>
              ))}
            </div>
          </fieldset>

          {/* Interaction */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">
              Interaction
            </legend>
            <div className="mt-2 space-y-2">
              {interactionFields.map((field, index) => (
                <div key={index} className="flex items-center gap-2">
                  <input
                    type="text"
                    value={field.key}
                    onChange={(e) => updateInteractionField(index, "key", e.target.value)}
                    placeholder="key"
                    className="w-40 rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
                  />
                  <span className="text-gray-400 text-sm">:</span>
                  <input
                    type="text"
                    value={field.value}
                    onChange={(e) => updateInteractionField(index, "value", e.target.value)}
                    placeholder="value"
                    className="flex-1 rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
                  />
                  <button
                    type="button"
                    onClick={() => removeInteractionField(index)}
                    className="text-xs text-red-400 hover:text-red-600"
                  >
                    Remove
                  </button>
                </div>
              ))}
              <button
                type="button"
                onClick={addInteractionField}
                className="text-sm text-indigo-600 hover:text-indigo-500"
              >
                + Add field
              </button>
            </div>
          </fieldset>
        </div>
      )}
    </div>
  );
}
