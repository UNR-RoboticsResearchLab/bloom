import React, { useState } from "react";
import LessonStepBuilder from "./LessonStepBuilder";

const LESSON_TYPES = [
  { value: 0, label: "Language" },
  { value: 1, label: "Speech Therapy" },
];

const inputClass =
  "block w-full rounded-md bg-white px-3 py-1.5 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
const textareaClass =
  "block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
const labelClass = "block text-sm font-medium text-gray-900";

function emptyStep(order) {
  return {
    _id: crypto.randomUUID(),
    stepOrder: order,
    type: "",
    script: "",
    timingSeconds: null,
    visualAid: null,
    behaviors: null,
    interaction: null,
  };
}

function parseObjectives(json) {
  if (!json) return [""];
  try {
    return JSON.parse(json);
  } catch {
    return [""];
  }
}

export default function LessonBuilder({ initialLesson = null, onSubmit, onCancel }) {
  const [title, setTitle] = useState(initialLesson?.title ?? "");
  const [description, setDescription] = useState(initialLesson?.description ?? "");
  const [lessonType, setLessonType] = useState(initialLesson?.lessonType ?? 0);
  const [objectives, setObjectives] = useState(() =>
    parseObjectives(initialLesson?.learningObjectives).map((text) => ({
      _id: crypto.randomUUID(),
      text,
    }))
  );
  const [steps, setSteps] = useState(() =>
    initialLesson?.steps?.length
      ? initialLesson.steps.map((s) => ({ ...s, _id: crypto.randomUUID() }))
      : [emptyStep(1)]
  );
  const [err, setErr] = useState("");
  const [submitting, setSubmitting] = useState(false);

  function updateObjective(id, text) {
    setObjectives((prev) => prev.map((o) => (o._id === id ? { ...o, text } : o)));
  }
  function addObjective() {
    setObjectives((prev) => [...prev, { _id: crypto.randomUUID(), text: "" }]);
  }
  function removeObjective(id) {
    setObjectives((prev) => prev.filter((o) => o._id !== id));
  }

  function addStep() {
    setSteps((prev) => [...prev, emptyStep(prev.length + 1)]);
  }
  function removeStep(localId) {
    setSteps((prev) =>
      prev.filter((s) => s._id !== localId).map((s, i) => ({ ...s, stepOrder: i + 1 }))
    );
  }
  function moveStep(localId, direction) {
    setSteps((prev) => {
      const idx = prev.findIndex((s) => s._id === localId);
      const swap = idx + direction;
      if (swap < 0 || swap >= prev.length) return prev;
      const next = [...prev];
      [next[idx], next[swap]] = [next[swap], next[idx]];
      return next.map((s, i) => ({ ...s, stepOrder: i + 1 }));
    });
  }
  function handleStepChange(localId, dto) {
    setSteps((prev) => prev.map((s) => (s._id === localId ? { ...s, ...dto } : s)));
  }

  async function handleSubmit(e) {
    e.preventDefault();
    setErr("");

    if (!title.trim()) {
      setErr("Lesson title is required.");
      return;
    }
    if (steps.some((s) => !s.type || !s.script?.trim())) {
      setErr("Every step needs a type and a script.");
      return;
    }

    setSubmitting(true);
    try {
      const filledObjectives = objectives.map((o) => o.text).filter((t) => t.trim());
      await onSubmit({
        ...(initialLesson?.id ? { id: initialLesson.id } : {}),
        title,
        description: description || null,
        lessonType,
        learningObjectives: filledObjectives.length ? JSON.stringify(filledObjectives) : null,
        steps: steps.map((s, i) => ({
          ...(s.id ? { id: s.id } : {}),
          stepOrder: i + 1,
          type: s.type,
          script: s.script,
          timingSeconds: s.timingSeconds,
          visualAid: s.visualAid,
          behaviors: s.behaviors,
          interaction: s.interaction,
        })),
      });
    } catch (error) {
      setErr(error.message || "Failed to save lesson.");
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div className="mx-auto max-w-5xl space-y-5 px-4 py-6">
      <div className="rounded-2xl border border-gray-100 bg-white px-5 py-4 shadow-sm">
        <h1 className="text-2xl font-semibold text-gray-900">
          {initialLesson ? "Edit Lesson" : "New Lesson"}
        </h1>
        <p className="mt-1 text-sm text-gray-500">
          Fill in the lesson details, then add and configure each step.
        </p>
      </div>

      {err && (
        <div className="rounded-md bg-red-50 px-4 py-3 text-sm text-red-600" role="alert">
          {err}
        </div>
      )}

      <form onSubmit={handleSubmit} className="space-y-8">
        <section className="rounded-2xl border border-gray-100 bg-white p-5 shadow-sm space-y-5">
          <h2 className="text-base font-semibold text-gray-800">Lesson Info</h2>

          <div>
            <label className={labelClass}>
              Title <span className="text-red-500">*</span>
            </label>
            <input
              type="text"
              value={title}
              onChange={(e) => setTitle(e.target.value)}
              required
              placeholder="Lesson title"
              className={`mt-2 ${inputClass}`}
            />
          </div>

          <div className="grid gap-4 sm:grid-cols-2">
            <div>
              <label className={labelClass}>Description</label>
              <textarea
                rows={3}
                value={description}
                onChange={(e) => setDescription(e.target.value)}
                placeholder="Optional description"
                className={`mt-2 ${textareaClass}`}
              />
            </div>

            <div>
              <label className={labelClass}>Lesson Type</label>
              <select
                value={lessonType}
                onChange={(e) => setLessonType(Number(e.target.value))}
                className={`mt-2 ${inputClass}`}
              >
                {LESSON_TYPES.map(({ value, label }) => (
                  <option key={value} value={value}>
                    {label}
                  </option>
                ))}
              </select>
            </div>
          </div>

          <div>
            <label className={labelClass}>Learning Objectives</label>
            <div className="mt-2 space-y-2">
              {objectives.map((obj) => (
                <div key={obj._id} className="flex items-center gap-2">
                  <input
                    type="text"
                    value={obj.text}
                    onChange={(e) => updateObjective(obj._id, e.target.value)}
                    placeholder={`Objective ${objectives.indexOf(obj) + 1}`}
                    className={`flex-1 ${inputClass}`}
                  />
                  <button
                    type="button"
                    onClick={() => removeObjective(obj._id)}
                    disabled={objectives.length === 1}
                    className="text-xs text-red-400 hover:text-red-600 disabled:opacity-30"
                  >
                    Remove
                  </button>
                </div>
              ))}
              <button
                type="button"
                onClick={addObjective}
                className="text-sm text-indigo-600 hover:text-indigo-500"
              >
                + Add objective
              </button>
            </div>
          </div>
        </section>

        <section className="space-y-2">
          <h2 className="text-base font-semibold text-gray-800">
            Steps{" "}
            <span className="ml-1 text-sm font-normal text-gray-400">({steps.length})</span>
          </h2>

          {steps.length === 0 && (
            <p className="rounded-lg border border-dashed border-gray-300 py-8 text-center text-sm text-gray-400">
              No steps yet. Add one below.
            </p>
          )}

          {steps.map((step, index) => (
            <LessonStepBuilder
              key={step._id}
              step={step}
              stepNumber={index + 1}
              isFirst={index === 0}
              isLast={index === steps.length - 1}
              onChange={(dto) => handleStepChange(step._id, dto)}
              onRemove={() => removeStep(step._id)}
              onMoveUp={() => moveStep(step._id, -1)}
              onMoveDown={() => moveStep(step._id, 1)}
            />
          ))}

          <button
            type="button"
            onClick={addStep}
            className="flex w-full items-center justify-center gap-2 rounded-2xl border border-dashed border-indigo-300 bg-white py-3 text-sm font-semibold text-indigo-600 shadow-sm transition hover:border-indigo-400 hover:bg-indigo-50"
          >
            + Add Step
          </button>
        </section>

        <div className="flex items-center justify-end gap-3 border-t border-gray-100 ">
          {onCancel && (
            <button
              type="button"
              onClick={onCancel}
              className="rounded-md px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-100"
            >
              Cancel
            </button>
          )}
          <button
            type="submit"
            disabled={submitting}
            className="flex justify-center rounded-md bg-indigo-600 px-5 py-2 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 disabled:cursor-not-allowed disabled:opacity-60 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
          >
            {submitting ? "Saving..." : initialLesson ? "Update Lesson" : "Save Lesson"}
          </button>
        </div>
      </form>
    </div>
  );
}
