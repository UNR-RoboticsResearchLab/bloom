import React, { useState } from "react";
import { useNavigate } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";
import { getSession } from "../utils/auth";

const STEP_TYPES = [
  "introduction", "presentation", "definition", "example",
  "interactive_question", "guided_practice", "summary", "closing",
  "loop", "conversation",
];

const MOTOR_SEQUENCES = [
  "excited", "happy", "calm", "idle", "thinking",
  "look_left", "look_right", "look_up", "bounce_smooth",
];

const BEHAVIOR_FIELDS = [
  { key: "behavior",          label: "Behavior",          options: ["excited", "happy", "calm", "idle", "thinking", "neutral", "invite_response"] },
  { key: "facial_expression", label: "Facial Expression", options: ["smile", "warm_smile", "playful", "engaged", "thoughtful", "neutral"] },
  { key: "gaze",              label: "Gaze",              options: ["center", "scan_room", "audience_member"] },
  { key: "head_movement",     label: "Head Movement",     options: null },
  { key: "posture",           label: "Posture",           options: ["lean_forward"] },
];

function newStep() {
  return {
    _id: crypto.randomUUID(),
    type: "",
    script: "",
    motor_sequence: "",
    timing_seconds: "",
    behaviors: { behavior: "", facial_expression: "", gaze: "", head_movement: "", posture: "" },
    visual_aid: [],
    interaction: {
      enabled: false,
      wait_for_response: true,
      single_turn_llm: false,
      max_wait_seconds: "10",
      correct_answer: "",
      correct_response_script: "",
      incorrect_response_script: "",
      single_turn_llm_prompt: "",
      fallback_script: "",
      fallback_visual_aid: [],
      fallback_visual_aid_labels: [],
    },
  };
}

function buildLessonJson(title, grade, objectives, steps) {
  const sequence = steps.map((s, i) => {
    const step = { id: i + 1, type: s.type, script: s.script };

    const beh = {};
    for (const { key } of BEHAVIOR_FIELDS) {
      if (s.behaviors[key]?.trim()) beh[key] = s.behaviors[key].trim();
    }
    if (Object.keys(beh).length) step.behaviors = beh;

    if (s.motor_sequence?.trim()) step.motor_sequence = s.motor_sequence.trim();
    if (s.timing_seconds !== "") step.timing_seconds = Number(s.timing_seconds);

    const filenames = s.visual_aid.map(v => v.filename).filter(Boolean);
    if (filenames.length) {
      step.visual_aid = filenames.length === 1 ? filenames[0] : filenames;
      const labels = s.visual_aid.map(v => v.label);
      const footers = s.visual_aid.map(v => v.footer);
      if (labels.some(l => l)) step.visual_aid_labels = labels;
      if (footers.some(f => f)) step.visual_aid_footers = footers;
    }

    if (s.interaction.enabled) {
      const intr = {};
      if (s.interaction.wait_for_response) intr.wait_for_response = true;
      if (s.interaction.max_wait_seconds !== "") intr.max_wait_seconds = Number(s.interaction.max_wait_seconds);

      if (s.interaction.single_turn_llm) {
        intr.single_turn_llm = true;
        if (s.interaction.single_turn_llm_prompt) intr.single_turn_llm_prompt = s.interaction.single_turn_llm_prompt;
      } else {
        if (s.interaction.correct_answer) intr.correct_answer = s.interaction.correct_answer;
        if (s.interaction.correct_response_script) intr.correct_response_script = s.interaction.correct_response_script;
        if (s.interaction.incorrect_response_script) intr.incorrect_response_script = s.interaction.incorrect_response_script;
      }

      if (s.interaction.fallback_script) intr.fallback_script = s.interaction.fallback_script;
      const fva = s.interaction.fallback_visual_aid.filter(Boolean);
      const fval = s.interaction.fallback_visual_aid_labels.filter(Boolean);
      if (fva.length) intr.fallback_visual_aid = fva;
      if (fval.length) intr.fallback_visual_aid_labels = fval;

      step.interaction = intr;
    }

    return step;
  });

  return {
    lesson: {
      title,
      grade: grade.trim() || "Elementary",
      objectives: objectives.filter(o => o.trim()),
    },
    sequence,
  };
}

// ---- StepCard ----

function StepCard({ step, stepNumber, onChange, onRemove, onMoveUp, onMoveDown, isFirst, isLast }) {
  const [expanded, setExpanded] = useState(!step.type);

  const inp = "block w-full rounded-md bg-white px-3 py-1.5 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
  const lbl = "block text-sm font-medium text-gray-700";
  const ta  = "block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
  const smInp = "block w-full rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";

  const set      = (field, val) => onChange({ ...step, [field]: val });
  const setBeh   = (key, val)   => onChange({ ...step, behaviors: { ...step.behaviors, [key]: val } });
  const setIntr  = (key, val)   => onChange({ ...step, interaction: { ...step.interaction, [key]: val } });

  const addVA    = ()       => set("visual_aid", [...step.visual_aid, { filename: "", label: "", footer: "" }]);
  const removeVA = (i)      => set("visual_aid", step.visual_aid.filter((_, j) => j !== i));
  const updateVA = (i, f, v) => set("visual_aid", step.visual_aid.map((va, j) => j === i ? { ...va, [f]: v } : va));

  const addFVA = () => onChange({
    ...step,
    interaction: {
      ...step.interaction,
      fallback_visual_aid: [...step.interaction.fallback_visual_aid, ""],
      fallback_visual_aid_labels: [...step.interaction.fallback_visual_aid_labels, ""],
    },
  });
  const removeFVA = (i) => onChange({
    ...step,
    interaction: {
      ...step.interaction,
      fallback_visual_aid: step.interaction.fallback_visual_aid.filter((_, j) => j !== i),
      fallback_visual_aid_labels: step.interaction.fallback_visual_aid_labels.filter((_, j) => j !== i),
    },
  });
  const updateFVA      = (i, v) => setIntr("fallback_visual_aid",        step.interaction.fallback_visual_aid.map((x, j) => j === i ? v : x));
  const updateFVALabel = (i, v) => setIntr("fallback_visual_aid_labels",  step.interaction.fallback_visual_aid_labels.map((x, j) => j === i ? v : x));

  const intr = step.interaction;
  const summary = step.type
    ? `${step.type}${step.script ? ` — ${step.script.slice(0, 55)}${step.script.length > 55 ? "…" : ""}` : ""}`
    : "New step";

  return (
    <div className="overflow-hidden rounded-2xl border border-gray-100 bg-white shadow-sm">
      {/* Header */}
      <div className="flex items-center gap-3 bg-gray-50/70 px-5 py-4">
        <span className="flex h-8 w-8 shrink-0 items-center justify-center rounded-full bg-indigo-100 text-sm font-semibold text-indigo-700">
          {stepNumber}
        </span>
        <button
          type="button"
          className="flex-1 truncate text-left text-sm font-medium text-gray-800 hover:text-indigo-600"
          onClick={() => setExpanded(v => !v)}
        >
          {summary}
        </button>
        <div className="flex shrink-0 items-center gap-1">
          <button type="button" onClick={onMoveUp}   disabled={isFirst} className="rounded p-1 text-gray-400 hover:bg-gray-100 disabled:opacity-30">↑</button>
          <button type="button" onClick={onMoveDown} disabled={isLast}  className="rounded p-1 text-gray-400 hover:bg-gray-100 disabled:opacity-30">↓</button>
          <button type="button" onClick={onRemove}                       className="rounded p-1 text-red-400 hover:bg-red-50">✕</button>
          <button type="button" onClick={() => setExpanded(v => !v)}     className="rounded p-1 text-gray-400 hover:bg-gray-100">
            {expanded ? "▲" : "▼"}
          </button>
        </div>
      </div>

      {expanded && (
        <div className="space-y-5 border-t border-gray-100 px-5 pb-5 pt-5">

          {/* Type / Motor Sequence / Timing */}
          <div className="grid gap-4 sm:grid-cols-3">
            <div>
              <label className={lbl}>Type <span className="text-red-500">*</span></label>
              <select value={step.type} onChange={e => set("type", e.target.value)} className={`mt-1 ${inp}`}>
                <option value="">Select type</option>
                {STEP_TYPES.map(t => <option key={t} value={t}>{t}</option>)}
              </select>
            </div>
            <div>
              <label className={lbl}>Motor Sequence</label>
              <select value={step.motor_sequence} onChange={e => set("motor_sequence", e.target.value)} className={`mt-1 ${inp}`}>
                <option value="">None</option>
                {MOTOR_SEQUENCES.map(m => <option key={m} value={m}>{m}</option>)}
              </select>
            </div>
            <div>
              <label className={lbl}>Timing (s)</label>
              <input
                type="number" min={0}
                value={step.timing_seconds}
                onChange={e => set("timing_seconds", e.target.value)}
                placeholder="Optional"
                className={`mt-1 ${inp}`}
              />
            </div>
          </div>

          {/* Script */}
          <div>
            <label className={lbl}>Script <span className="text-red-500">*</span></label>
            <textarea
              rows={3}
              value={step.script}
              onChange={e => set("script", e.target.value)}
              placeholder="What Bloom says..."
              className={`mt-1 ${ta}`}
            />
          </div>

          {/* Behaviors */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">Behaviors</legend>
            <div className="mt-2 grid gap-3 sm:grid-cols-3">
              {BEHAVIOR_FIELDS.map(({ key, label, options }) => (
                <div key={key}>
                  <label className="block text-xs font-medium text-gray-600">{label}</label>
                  {options ? (
                    <select value={step.behaviors[key]} onChange={e => setBeh(key, e.target.value)} className={`mt-1 ${smInp}`}>
                      <option value="">None</option>
                      {options.map(o => <option key={o} value={o}>{o}</option>)}
                    </select>
                  ) : (
                    <input type="text" value={step.behaviors[key]} onChange={e => setBeh(key, e.target.value)} placeholder="Optional" className={`mt-1 ${smInp}`} />
                  )}
                </div>
              ))}
            </div>
          </fieldset>

          {/* Visual Aid */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">Visual Aid</legend>
            <div className="mt-2 space-y-2">
              {step.visual_aid.length > 0 && (
                <div className="grid grid-cols-3 gap-2 px-1 text-xs font-medium text-gray-400">
                  <span>Filename</span><span>Label</span><span>Footer</span>
                </div>
              )}
              {step.visual_aid.map((v, i) => (
                <div key={i} className="grid gap-2 sm:grid-cols-3">
                  <input type="text" value={v.filename} onChange={e => updateVA(i, "filename", e.target.value)} placeholder="image.png" className={smInp} />
                  <input type="text" value={v.label}    onChange={e => updateVA(i, "label",    e.target.value)} placeholder="Label (optional)" className={smInp} />
                  <div className="flex gap-2">
                    <input type="text" value={v.footer} onChange={e => updateVA(i, "footer", e.target.value)} placeholder="Footer (optional)"
                      className="flex-1 rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600" />
                    <button type="button" onClick={() => removeVA(i)} className="shrink-0 text-xs text-red-400 hover:text-red-600">Remove</button>
                  </div>
                </div>
              ))}
              <button type="button" onClick={addVA} className="text-sm text-indigo-600 hover:text-indigo-500">+ Add image</button>
            </div>
          </fieldset>

          {/* Interaction */}
          <fieldset className="rounded-lg border border-gray-100 p-3">
            <legend className="px-1 text-xs font-semibold uppercase tracking-wide text-gray-500">Interaction</legend>
            <div className="mt-3">
              <label className="flex cursor-pointer items-center gap-2 text-sm text-gray-700">
                <input
                  type="checkbox"
                  checked={intr.enabled}
                  onChange={e => setIntr("enabled", e.target.checked)}
                  className="rounded border-gray-300 text-indigo-600"
                />
                Enable student interaction
              </label>

              {intr.enabled && (
                <div className="mt-4 space-y-4 border-t border-gray-100 pt-4">

                  {/* Flags */}
                  <div className="flex flex-wrap gap-6">
                    <label className="flex cursor-pointer items-center gap-2 text-sm text-gray-700">
                      <input type="checkbox" checked={intr.wait_for_response} onChange={e => setIntr("wait_for_response", e.target.checked)} className="rounded border-gray-300 text-indigo-600" />
                      Wait for response
                    </label>
                    <label className="flex cursor-pointer items-center gap-2 text-sm text-gray-700">
                      <input type="checkbox" checked={intr.single_turn_llm} onChange={e => setIntr("single_turn_llm", e.target.checked)} className="rounded border-gray-300 text-indigo-600" />
                      Use LLM evaluation
                    </label>
                  </div>

                  {/* Max wait + correct answer (when no LLM) */}
                  <div className="grid gap-4 sm:grid-cols-2">
                    <div>
                      <label className={lbl}>Max Wait (seconds)</label>
                      <input type="number" min={0} value={intr.max_wait_seconds} onChange={e => setIntr("max_wait_seconds", e.target.value)} className={`mt-1 ${inp}`} />
                    </div>
                    {!intr.single_turn_llm && (
                      <div>
                        <label className={lbl}>Correct Answer</label>
                        <input type="text" value={intr.correct_answer} onChange={e => setIntr("correct_answer", e.target.value)} placeholder='e.g. "one" or "sea"' className={`mt-1 ${inp}`} />
                      </div>
                    )}
                  </div>

                  {/* LLM prompt or correct/incorrect scripts */}
                  {intr.single_turn_llm ? (
                    <div>
                      <label className={lbl}>LLM Evaluation Prompt</label>
                      <textarea
                        rows={4}
                        value={intr.single_turn_llm_prompt}
                        onChange={e => setIntr("single_turn_llm_prompt", e.target.value)}
                        placeholder="Instructions for the LLM to evaluate the student's response..."
                        className={`mt-1 ${ta}`}
                      />
                    </div>
                  ) : (
                    <div className="grid gap-4 sm:grid-cols-2">
                      <div>
                        <label className={lbl}>Correct Response Script</label>
                        <textarea rows={2} value={intr.correct_response_script} onChange={e => setIntr("correct_response_script", e.target.value)} placeholder="What Bloom says if correct..." className={`mt-1 ${ta}`} />
                      </div>
                      <div>
                        <label className={lbl}>Incorrect Response Script</label>
                        <textarea rows={2} value={intr.incorrect_response_script} onChange={e => setIntr("incorrect_response_script", e.target.value)} placeholder="What Bloom says if incorrect..." className={`mt-1 ${ta}`} />
                      </div>
                    </div>
                  )}

                  {/* Fallback script */}
                  <div>
                    <label className={lbl}>Fallback Script</label>
                    <textarea rows={2} value={intr.fallback_script} onChange={e => setIntr("fallback_script", e.target.value)} placeholder="What Bloom says if no response..." className={`mt-1 ${ta}`} />
                  </div>

                  {/* Fallback visual aid */}
                  <div>
                    <label className={lbl}>Fallback Visual Aid</label>
                    <div className="mt-1 space-y-2">
                      {intr.fallback_visual_aid.map((va, i) => (
                        <div key={i} className="flex gap-2">
                          <input type="text" value={va} onChange={e => updateFVA(i, e.target.value)} placeholder="image.png"
                            className="flex-1 rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600" />
                          <input type="text" value={intr.fallback_visual_aid_labels[i] ?? ""} onChange={e => updateFVALabel(i, e.target.value)} placeholder="Label"
                            className="w-32 rounded-md bg-white px-2 py-1 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600" />
                          <button type="button" onClick={() => removeFVA(i)} className="shrink-0 text-xs text-red-400 hover:text-red-600">Remove</button>
                        </div>
                      ))}
                      <button type="button" onClick={addFVA} className="text-sm text-indigo-600 hover:text-indigo-500">+ Add fallback image</button>
                    </div>
                  </div>
                </div>
              )}
            </div>
          </fieldset>
        </div>
      )}
    </div>
  );
}

// ---- Main Component ----

export default function AddLessonCard() {
  const navigate = useNavigate();
  const api = useApiClient();

  const [title,      setTitle]      = useState("");
  const [grade,      setGrade]      = useState("");
  const [description, setDescription] = useState("");
  const [lessonType, setLessonType] = useState("Language");
  const [objectives, setObjectives] = useState([""]);
  const [steps,      setSteps]      = useState([newStep()]);
  const [err,        setErr]        = useState("");
  const [submitting, setSubmitting] = useState(false);
  const [success,    setSuccess]    = useState(false);

  const updateObjective = (i, val) => setObjectives(prev => prev.map((o, j) => j === i ? val : o));
  const addObjective    = ()       => setObjectives(prev => [...prev, ""]);
  const removeObjective = (i)      => setObjectives(prev => prev.filter((_, j) => j !== i));

  const addStep    = ()      => setSteps(prev => [...prev, newStep()]);
  const removeStep = (id)    => setSteps(prev => prev.filter(s => s._id !== id));
  const updateStep = (id, s) => setSteps(prev => prev.map(x => x._id === id ? s : x));
  const moveStep   = (id, dir) => setSteps(prev => {
    const idx = prev.findIndex(s => s._id === id);
    const swap = idx + dir;
    if (swap < 0 || swap >= prev.length) return prev;
    const next = [...prev];
    [next[idx], next[swap]] = [next[swap], next[idx]];
    return next;
  });

  async function handleSubmit(e) {
    e.preventDefault();
    setErr("");

    if (!title.trim())                                        { setErr("Title is required.");                        return; }
    if (steps.length === 0)                                   { setErr("Add at least one step.");                    return; }
    if (steps.some(s => !s.type || !s.script.trim()))         { setErr("Every step needs a type and a script.");     return; }

    setSubmitting(true);
    try {
      const session = getSession();
      if (!session?.id) { setErr("Unable to determine user ID. Please sign in again."); return; }

      const lessonJson = buildLessonJson(title.trim(), grade, objectives, steps);

      const response = await api.createLesson({
        title: title.trim(),
        description: description.trim() || null,
        lessonType: lessonType === "Language" ? 0 : 1,
        lessonDescription: JSON.stringify(lessonJson),
        createdById: session.id,
      });

      if (response?.message?.includes("successfully")) {
        setSuccess(true);
        setTimeout(() => navigate("/dashboard/admin"), 1500);
      } else {
        setErr(response?.message || "Failed to create lesson.");
      }
    } catch (error) {
      setErr(error.message || "An error occurred while creating the lesson.");
    } finally {
      setSubmitting(false);
    }
  }

  const labelClass = "block text-sm font-medium text-gray-900";
  const inputClass = "block w-full rounded-md bg-white px-3 py-1.5 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
  const taClass    = "block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";

  return (
    <div className="mx-auto max-w-5xl space-y-5 px-4 py-6">

      {/* Page header */}
      <div className="rounded-2xl border border-gray-100 bg-white px-5 py-4 shadow-sm">
        <h1 className="text-2xl font-semibold text-gray-900">Add Lesson</h1>
        <p className="mt-1 text-sm text-gray-500">Build a step-by-step lesson for Bloom.</p>
      </div>

      {err     && <div className="rounded-md bg-red-50   px-4 py-3 text-sm text-red-600"   role="alert">{err}</div>}
      {success && <div className="rounded-md bg-green-50 px-4 py-3 text-sm text-green-600">Lesson created successfully! Redirecting...</div>}

      <form onSubmit={handleSubmit} className="space-y-6">

        {/* Lesson info */}
        <section className="space-y-5 rounded-2xl border border-gray-100 bg-white p-5 shadow-sm">
          <h2 className="text-base font-semibold text-gray-800">Lesson Info</h2>

          <div className="grid gap-4 sm:grid-cols-2">
            <div>
              <label className={labelClass}>Title <span className="text-red-500">*</span></label>
              <input type="text" value={title} onChange={e => setTitle(e.target.value)} required placeholder="Lesson title" className={`mt-2 ${inputClass}`} />
            </div>
            <div>
              <label className={labelClass}>Grade</label>
              <input type="text" value={grade} onChange={e => setGrade(e.target.value)} placeholder="e.g. Elementary" className={`mt-2 ${inputClass}`} />
            </div>
          </div>

          <div className="grid gap-4 sm:grid-cols-2">
            <div>
              <label className={labelClass}>Description</label>
              <textarea rows={3} value={description} onChange={e => setDescription(e.target.value)} placeholder="Optional description" className={`mt-2 ${taClass}`} />
            </div>
            <div>
              <label className={labelClass}>Lesson Type</label>
              <select value={lessonType} onChange={e => setLessonType(e.target.value)} className={`mt-2 ${inputClass}`}>
                <option value="Language">Language</option>
                <option value="SpeechTherapy">Speech Therapy</option>
              </select>
            </div>
          </div>

          {/* Objectives */}
          <div>
            <label className={labelClass}>Learning Objectives</label>
            <div className="mt-2 space-y-2">
              {objectives.map((obj, i) => (
                <div key={i} className="flex items-center gap-2">
                  <input
                    type="text"
                    value={obj}
                    onChange={e => updateObjective(i, e.target.value)}
                    placeholder={`Objective ${i + 1}`}
                    className={`flex-1 ${inputClass}`}
                  />
                  <button
                    type="button"
                    onClick={() => removeObjective(i)}
                    disabled={objectives.length === 1}
                    className="text-xs text-red-400 hover:text-red-600 disabled:opacity-30"
                  >
                    Remove
                  </button>
                </div>
              ))}
              <button type="button" onClick={addObjective} className="text-sm text-indigo-600 hover:text-indigo-500">
                + Add objective
              </button>
            </div>
          </div>
        </section>

        {/* Steps */}
        <section className="space-y-3">
          <h2 className="text-base font-semibold text-gray-800">
            Steps <span className="ml-1 text-sm font-normal text-gray-400">({steps.length})</span>
          </h2>

          {steps.length === 0 && (
            <p className="rounded-lg border border-dashed border-gray-300 py-8 text-center text-sm text-gray-400">
              No steps yet. Add one below.
            </p>
          )}

          {steps.map((step, index) => (
            <StepCard
              key={step._id}
              step={step}
              stepNumber={index + 1}
              isFirst={index === 0}
              isLast={index === steps.length - 1}
              onChange={s => updateStep(step._id, s)}
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

        {/* Actions */}
        <div className="flex items-center justify-end gap-3 border-t border-gray-100 pt-4">
          <button
            type="button"
            onClick={() => navigate("/dashboard/admin")}
            className="rounded-md px-4 py-2 text-sm font-medium text-gray-700 hover:bg-gray-100"
          >
            Cancel
          </button>
          <button
            type="submit"
            disabled={submitting}
            className="rounded-md bg-indigo-600 px-5 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500 disabled:opacity-50"
          >
            {submitting ? "Creating..." : "Create Lesson"}
          </button>
        </div>
      </form>
    </div>
  );
}
