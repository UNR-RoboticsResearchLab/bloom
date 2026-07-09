// LessonAiPromptPanel.jsx
// Inline panel (shown below Lesson Info when the AI Assistant switch is on) that lets
// an SLP fine-tune a prompt before generating a lesson with AI.
// Structured fields are a convenience for composing text — the prompt textarea
// is the actual source of truth submitted via onGenerate.

import React, { useState } from "react";

const inputClass =
  "block w-full rounded-md bg-white px-3 py-1.5 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
const textareaClass =
  "block w-full rounded-md bg-white px-3 py-2 text-sm text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600";
const labelClass = "block text-sm font-medium text-gray-900";

function composeDetails({ topic, audience, stepCount, notes }) {
  const parts = [];
  if (topic.trim()) parts.push(topic.trim());
  if (audience.trim()) parts.push(`Audience: ${audience.trim()}.`);
  if (stepCount.trim()) parts.push(`Aim for about ${stepCount.trim()} steps.`);
  if (notes.trim()) parts.push(notes.trim());
  return parts.join(" ");
}

export default function LessonAiPromptPanel({ initialPrompt = "", onGenerate }) {
  const [topic, setTopic] = useState("");
  const [audience, setAudience] = useState("");
  const [stepCount, setStepCount] = useState("");
  const [notes, setNotes] = useState("");
  const [prompt, setPrompt] = useState(initialPrompt);
  const [submitting, setSubmitting] = useState(false);
  const [err, setErr] = useState("");

  function applyDetailsToPrompt() {
    const composed = composeDetails({ topic, audience, stepCount, notes });
    if (!composed) return;
    setPrompt((prev) => (prev.trim() ? `${prev.trim()}\n${composed}` : composed));
    setTopic("");
    setAudience("");
    setStepCount("");
    setNotes("");
  }

  async function handleGenerate() {
    if (!prompt.trim()) {
      setErr("Describe the lesson you want before generating.");
      return;
    }
    setErr("");
    setSubmitting(true);
    try {
      await onGenerate(prompt.trim());
    } catch (error) {
      setErr(error.message || "Failed to generate lesson.");
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <section className="space-y-5 rounded-2xl border border-indigo-100 bg-indigo-50/40 p-5 shadow-sm">
      <div>
        <h2 className="text-base font-semibold text-gray-800">AI Assistant</h2>
        <p className="mt-1 text-sm text-gray-500">
          Fill in a few details to fine-tune a prompt alongside the existing lesson title, description, type, and learning objectives - or just write it directly below, then generate!
        </p>
      </div>

      <div className="grid gap-4 sm:grid-cols-2">
        <div>
          <label className={labelClass}>Focus / topic</label>
          <input
            type="text"
            value={topic}
            onChange={(e) => setTopic(e.target.value)}
            placeholder="e.g. /r/ sound practice"
            className={`mt-2 ${inputClass}`}
          />
        </div>
        <div>
          <label className={labelClass}>Audience</label>
          <input
            type="text"
            value={audience}
            onChange={(e) => setAudience(e.target.value)}
            placeholder="e.g. 7-9 year olds"
            className={`mt-2 ${inputClass}`}
          />
        </div>
        <div>
          <label className={labelClass}>Number of steps</label>
          <input
            type="number"
            min={1}
            max={20}
            value={stepCount}
            onChange={(e) => setStepCount(e.target.value)}
            placeholder="Optional"
            className={`mt-2 ${inputClass}`}
          />
        </div>
        <div className="sm:col-span-2">
          <label className={labelClass}>Other notes</label>
          <input
            type="text"
            value={notes}
            onChange={(e) => setNotes(e.target.value)}
            placeholder="e.g. include a question step, keep it playful"
            className={`mt-2 ${inputClass}`}
          />
        </div>
      </div>

      <button
        type="button"
        onClick={applyDetailsToPrompt}
        className="text-sm text-indigo-600 hover:text-indigo-500"
      >
        + Add these details to the prompt
      </button>

      <div>
        <label className={labelClass}>Prompt</label>
        <textarea
          rows={4}
          value={prompt}
          onChange={(e) => setPrompt(e.target.value)}
          placeholder="Describe the lesson you want..."
          className={`mt-2 ${textareaClass}`}
        />
      </div>

      {err && (
        <div className="rounded-md bg-red-50 px-4 py-3 text-sm text-red-600" role="alert">
          {err}
        </div>
      )}

      <div className="flex justify-end">
        <button
          type="button"
          onClick={handleGenerate}
          disabled={submitting}
          className="flex justify-center rounded-md bg-indigo-600 px-5 py-2 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 disabled:cursor-not-allowed disabled:opacity-60 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
        >
          {submitting ? "Generating..." : "Generate with AI"}
        </button>
      </div>
    </section>
  );
}
