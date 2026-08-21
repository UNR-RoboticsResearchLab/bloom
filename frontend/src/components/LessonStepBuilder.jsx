// LessonStepBuilder.jsx
// Collapsible card for editing a single LessonStep within the LessonBuilder.
// Calls onChange(stepDto) whenever the step state changes.

import React, { useMemo, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";
import ToggleSwitch from "./ToggleSwitch";
import VisualAidField from "./VisualAidField";

export const STEP_TYPES = [
  "instruction",
  "introduction",
  "question",
  "wait_for_response",
  "feedback",
  "transition",
  "conclusion",
];

// Maps each behaviors-object key to the Behavior entity field it's derived from.
// defaultOptions are shown even when the API returns no data.
const BEHAVIOR_FIELDS = [
  { key: "behavior", label: "Behavior", optionField: "name", defaultOptions: ["calm", "excited", "happy", "idle", "thinking"] },
  { key: "facial_expression", label: "Facial Expression", optionField: "facialExpression", defaultOptions: ["happy", "sad", "surprised", "neutral", "angry"] },
  { key: "gaze", label: "Gaze", optionField: "gaze", defaultOptions: ["forward", "left", "right", "up", "down"] },
  { key: "head_movement", label: "Head Movement", optionField: "headMovement", defaultOptions: ["nod", "shake", "tilt_left", "tilt_right", "still"] },
];

// Step types that usually expect a student response, used to auto-enable
// "Wait for a response" the first time one of these types is picked.
const AUTO_WAIT_TYPES = new Set(["question", "wait_for_response"]);
const DEFAULT_MAX_WAIT_SECONDS = 10;

// The full, known field set for a step's interaction config — mirrors the
// backend's StepInteractionDto field-for-field (camelCase).
const INTERACTION_KEYS = [
  "waitForResponse",
  "maxWaitSeconds",
  "correctAnswer",
  "correctResponseScript",
  "incorrectResponseScript",
  "singleTurnLlm",
  "singleTurnLlmPrompt",
  "llmFollowUp",
  "fallbackScript",
  "fallbackVisualAid",
  "fallbackVisualAidLabels",
];

function emptyInteraction() {
  return {
    waitForResponse: false,
    maxWaitSeconds: null,
    correctAnswer: "",
    correctResponseScript: "",
    incorrectResponseScript: "",
    singleTurnLlm: false,
    singleTurnLlmPrompt: "",
    llmFollowUp: false,
    fallbackScript: "",
    fallbackVisualAid: "",
    fallbackVisualAidLabels: "",
  };
}

const PRISTINE_INTERACTION_JSON = JSON.stringify(emptyInteraction());

function isPristineInteraction(interaction) {
  return JSON.stringify(interaction) === PRISTINE_INTERACTION_JSON;
}

function parseBehaviors(value) {
  const defaults = { behavior: "", facial_expression: "", gaze: "", head_movement: "" };
  if (!value) return defaults;
  if (typeof value === "string") {
    try { return { ...defaults, ...JSON.parse(value) }; }
    catch { return defaults; }
  }
  return { ...defaults, ...value };
}

// Accepts the interaction as the API returns it (a plain object), a legacy
// JSON string, or nothing at all — and always returns a fully-populated,
// typed object. Unknown/legacy keys (e.g. from the old free-form editor) are
// silently dropped rather than crashing or leaking into the saved DTO.
function parseInteraction(value) {
  const defaults = emptyInteraction();
  if (!value) return defaults;

  let obj = value;
  if (typeof value === "string") {
    try { obj = JSON.parse(value); }
    catch { return defaults; }
  }
  if (typeof obj !== "object" || obj === null) return defaults;

  const result = { ...defaults };
  for (const key of INTERACTION_KEYS) {
    if (obj[key] !== undefined && obj[key] !== null) result[key] = obj[key];
  }
  result.waitForResponse = Boolean(result.waitForResponse);
  result.singleTurnLlm = Boolean(result.singleTurnLlm);
  result.llmFollowUp = Boolean(result.llmFollowUp);
  result.maxWaitSeconds =
    result.maxWaitSeconds === "" || result.maxWaitSeconds == null ? null : Number(result.maxWaitSeconds);
  return result;
}

// Builds the StepInteractionDto payload, or null when there's nothing worth saving.
function buildInteractionDto(interaction) {
  const hasContent =
    interaction.waitForResponse ||
    interaction.maxWaitSeconds != null ||
    interaction.correctAnswer.trim() !== "" ||
    interaction.correctResponseScript.trim() !== "" ||
    interaction.incorrectResponseScript.trim() !== "" ||
    interaction.singleTurnLlm ||
    interaction.singleTurnLlmPrompt.trim() !== "" ||
    interaction.llmFollowUp ||
    interaction.fallbackScript.trim() !== "" ||
    interaction.fallbackVisualAid !== "" ||
    interaction.fallbackVisualAidLabels !== "";

  if (!hasContent) return null;

  return {
    waitForResponse: Boolean(interaction.waitForResponse),
    maxWaitSeconds:
      interaction.maxWaitSeconds === "" || interaction.maxWaitSeconds == null
        ? null
        : Number(interaction.maxWaitSeconds),
    correctAnswer: interaction.correctAnswer.trim() || null,
    correctResponseScript: interaction.correctResponseScript.trim() || null,
    incorrectResponseScript: interaction.incorrectResponseScript.trim() || null,
    singleTurnLlm: Boolean(interaction.singleTurnLlm),
    singleTurnLlmPrompt: interaction.singleTurnLlmPrompt.trim() || null,
    llmFollowUp: Boolean(interaction.llmFollowUp),
    fallbackScript: interaction.fallbackScript.trim() || null,
    fallbackVisualAid: interaction.fallbackVisualAid || null,
    fallbackVisualAidLabels: interaction.fallbackVisualAidLabels || null,
  };
}

export function buildStepDto(stepOrder, core, behaviors, interaction, existingId) {
  const behaviorsObj = Object.fromEntries(
    Object.entries(behaviors).filter(([, v]) => v.trim() !== "")
  );

  return {
    ...(existingId ? { id: existingId } : {}),
    stepOrder,
    title: core.title || null,
    type: core.type,
    script: core.script,
    timingSeconds: core.timingSeconds !== "" ? Number(core.timingSeconds) : null,
    visualAid: core.visualAid || null,
    motorSequence: core.motorSequence || null,
    behaviors: Object.keys(behaviorsObj).length > 0 ? behaviorsObj : null,
    interaction: buildInteractionDto(interaction),
  };
}

export default function LessonStepBuilder({
  step,
  stepNumber,
  onChange,
  onRemove,
  onMoveUp,
  onMoveDown,
  isFirst,
  isLast,
  behaviorOptions = [],
  motorSequenceOptions = [],
}) {
  const api = useApiClient();
  const [expanded, setExpanded] = useState(!step.type);

  const [core, setCore] = useState({
    title: step.title ?? "",
    type: step.type ?? "",
    script: step.script ?? "",
    timingSeconds: step.timingSeconds ?? "",
    visualAid: step.visualAid ?? "",
    motorSequence: step.motorSequence ?? "",
  });

  const [behaviors, setBehaviors] = useState(() => parseBehaviors(step.behaviors));
  const [interaction, setInteraction] = useState(() => parseInteraction(step.interaction));

  const behaviorFieldOptions = useMemo(() => {
    const result = {};
    for (const { key, optionField, defaultOptions } of BEHAVIOR_FIELDS) {
      const fromApi = behaviorOptions.map((b) => b[optionField]).filter(Boolean);
      result[key] = [...new Set([...defaultOptions, ...fromApi])].sort();
    }
    return result;
  }, [behaviorOptions]);

  function emit(nextCore, nextBehaviors, nextInteraction) {
    const dto = buildStepDto(stepNumber, nextCore, nextBehaviors, nextInteraction, step.id);
    onChange(dto);
  }

  function handleCoreChange(field, value) {
    const next = { ...core, [field]: value };
    setCore(next);

    if (field === "type" && AUTO_WAIT_TYPES.has(value) && isPristineInteraction(interaction)) {
      const seeded = { ...interaction, waitForResponse: true, maxWaitSeconds: DEFAULT_MAX_WAIT_SECONDS };
      setInteraction(seeded);
      emit(next, behaviors, seeded);
    } else {
      emit(next, behaviors, interaction);
    }
  }

  function handleBehaviorChange(key, value) {
    const next = { ...behaviors, [key]: value };
    setBehaviors(next);
    emit(core, next, interaction);
  }

  function handleInteractionChange(field, value) {
    const next = { ...interaction, [field]: value };
    setInteraction(next);
    emit(core, behaviors, next);
  }

  const MAX_VISUAL_AIDS = 4;

  const inputClass =
    "block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm";
  const textareaClass =
    "block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
  const labelClass = "block text-sm font-medium text-gray-700";
  const helperClass = "text-xs text-gray-400";

  const summary = core.title
    ? core.title
    : core.type
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

          {/* Title */}
          <div>
            <label className={labelClass}>Title</label>
            <div className="mt-1">
              <input
                type="text"
                value={core.title}
                onChange={(e) => handleCoreChange("title", e.target.value)}
                placeholder="Optional step title"
                className={inputClass}
              />
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
                className={textareaClass}
              />
            </div>
          </div>

          {/* Visual Aid */}
          <div>
            <label className={labelClass}>Visual Aid</label>
            <VisualAidField
              value={core.visualAid}
              onChange={(v) => handleCoreChange("visualAid", v)}
              max={MAX_VISUAL_AIDS}
              api={api}
            />
          </div>

          {/* Behaviors */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">
              Behaviors
            </legend>
            <div className="mt-2 grid gap-3 sm:grid-cols-2 lg:grid-cols-3">
              {BEHAVIOR_FIELDS.map(({ key, label }) => {
                const options = behaviorFieldOptions[key] ?? [];
                return (
                  <div key={key}>
                    <label className="block text-xs font-medium text-gray-600">{label}</label>
                    {options.length > 0 ? (
                      <select
                        value={behaviors[key]}
                        onChange={(e) => handleBehaviorChange(key, e.target.value)}
                        className="mt-1 block w-full rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
                      >
                        <option value="">None</option>
                        {options.map((opt) => (
                          <option key={opt} value={opt}>
                            {opt}
                          </option>
                        ))}
                      </select>
                    ) : (
                      <input
                        type="text"
                        value={behaviors[key]}
                        onChange={(e) => handleBehaviorChange(key, e.target.value)}
                        placeholder="Optional"
                        className="mt-1 block w-full rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600"
                      />
                    )}
                  </div>
                );
              })}
            </div>
          </fieldset>

          {/* Interaction */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">
              Interaction
            </legend>

            <div className="mt-2 flex items-center justify-between gap-3">
              <div>
                <label className={labelClass}>Wait for a response</label>
                <p className={helperClass}>Pause after the script and listen before moving on.</p>
              </div>
              <ToggleSwitch
                checked={interaction.waitForResponse}
                onChange={(v) => handleInteractionChange("waitForResponse", v)}
                ariaLabel="Toggle wait for response"
              />
            </div>

            {interaction.waitForResponse && (
              <div className="mt-4 space-y-4 border-t border-gray-100 pt-4">
                <div>
                  <label className={labelClass}>Max wait time (seconds)</label>
                  <p className={`${helperClass} mb-1`}>How long to wait before giving up.</p>
                  <input
                    type="number"
                    min={0}
                    value={interaction.maxWaitSeconds ?? ""}
                    onChange={(e) =>
                      handleInteractionChange(
                        "maxWaitSeconds",
                        e.target.value === "" ? null : e.target.value
                      )
                    }
                    placeholder="Optional"
                    className={inputClass}
                  />
                </div>

                <div>
                  <label className={labelClass}>Expected answer</label>
                  <p className={`${helperClass} mb-1`}>
                    Text the robot compares against what the student says.
                  </p>
                  <input
                    type="text"
                    value={interaction.correctAnswer}
                    onChange={(e) => handleInteractionChange("correctAnswer", e.target.value)}
                    placeholder="e.g. blue"
                    className={inputClass}
                  />
                </div>

                <div>
                  <label className={labelClass}>If correct, robot says…</label>
                  <textarea
                    rows={2}
                    value={interaction.correctResponseScript}
                    onChange={(e) => handleInteractionChange("correctResponseScript", e.target.value)}
                    className={textareaClass}
                  />
                </div>

                <div>
                  <label className={labelClass}>If incorrect, robot says…</label>
                  <textarea
                    rows={2}
                    value={interaction.incorrectResponseScript}
                    onChange={(e) => handleInteractionChange("incorrectResponseScript", e.target.value)}
                    className={textareaClass}
                  />
                </div>

                <div className="rounded-md bg-gray-50 p-3 space-y-3">
                  <div className="flex items-center justify-between gap-3">
                    <div>
                      <label className={labelClass}>Use AI to judge the answer</label>
                      <p className={helperClass}>
                        Instead of an exact text match, let AI decide if the student's spoken
                        answer means the same thing as the expected answer.
                      </p>
                    </div>
                    <ToggleSwitch
                      checked={interaction.singleTurnLlm}
                      onChange={(v) => handleInteractionChange("singleTurnLlm", v)}
                      ariaLabel="Toggle AI answer judging"
                    />
                  </div>

                  {interaction.singleTurnLlm && (
                    <div>
                      <label className={labelClass}>AI evaluation instructions</label>
                      <p className={`${helperClass} mb-1`}>Optional — leave blank to use the default.</p>
                      <textarea
                        rows={2}
                        value={interaction.singleTurnLlmPrompt}
                        onChange={(e) => handleInteractionChange("singleTurnLlmPrompt", e.target.value)}
                        className={textareaClass}
                      />
                    </div>
                  )}

                  <div className="flex items-center justify-between gap-3">
                    <div>
                      <label className={labelClass}>Allow a follow-up exchange</label>
                      <p className={helperClass}>
                        Let the robot ask one more clarifying question instead of stopping after a
                        single reply.
                      </p>
                    </div>
                    <ToggleSwitch
                      checked={interaction.llmFollowUp}
                      onChange={(v) => handleInteractionChange("llmFollowUp", v)}
                      ariaLabel="Toggle AI follow-up"
                    />
                  </div>
                </div>

                <fieldset className="rounded-md border border-gray-100 p-3">
                  <legend className="px-1 text-xs font-semibold text-gray-500">
                    If there's no response
                  </legend>
                  <div className="mt-2 space-y-3">
                    <div>
                      <label className={labelClass}>Fallback script</label>
                      <p className={`${helperClass} mb-1`}>
                        What the robot says if the student doesn't answer in time.
                      </p>
                      <textarea
                        rows={2}
                        value={interaction.fallbackScript}
                        onChange={(e) => handleInteractionChange("fallbackScript", e.target.value)}
                        className={textareaClass}
                      />
                    </div>
                    <div>
                      <label className={labelClass}>Fallback image</label>
                      <VisualAidField
                        value={interaction.fallbackVisualAid}
                        onChange={(v) => handleInteractionChange("fallbackVisualAid", v)}
                        max={1}
                        api={api}
                      />
                    </div>
                  </div>
                </fieldset>
              </div>
            )}
          </fieldset>
        </div>
      )}
    </div>
  );
}
