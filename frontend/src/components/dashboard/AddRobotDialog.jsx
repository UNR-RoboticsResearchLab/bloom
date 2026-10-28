// AddRobotDialog.jsx
// Admin flow for adding a robot: name + model -> POST /api/robot/register -> the robot's login
// secret, which the backend shows exactly once. The second step therefore can't be dismissed by
// accident (no backdrop click, Escape ignored) until the admin confirms they've saved it.
//
// Accessible: a modal dialog (role/aria-modal/labelled), focus moves in and is trapped, Escape
// closes it where that's safe, focus returns to the opener, errors are announced via aria-live,
// and every control is reachable and operable by keyboard.

import React, { useEffect, useId, useRef, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";

const MODEL_SUGGESTIONS = ["Blossom"];
const FOCUSABLE =
  'a[href], button:not([disabled]), input:not([disabled]), select:not([disabled]), textarea:not([disabled]), [tabindex]:not([tabindex="-1"])';

export default function AddRobotDialog({ onClose, onCreated }) {
  const api = useApiClient();
  const ids = { title: useId(), desc: useId(), error: useId(), secret: useId() };
  const dialogRef = useRef(null);
  const nameRef = useRef(null);
  const openerRef = useRef(document.activeElement);

  const [name, setName] = useState("");
  const [model, setModel] = useState(MODEL_SUGGESTIONS[0]);
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState("");
  const [created, setCreated] = useState(null); // { robot, secret }
  const [saved, setSaved] = useState(false);
  const [copied, setCopied] = useState(false);

  // Secret not yet acknowledged: closing would lose it forever.
  const locked = !!created && !saved;

  useEffect(() => {
    nameRef.current?.focus();
    const opener = openerRef.current;
    return () => opener?.focus?.();
  }, []);

  function handleKeyDown(e) {
    if (e.key === "Escape" && !locked && !submitting) {
      e.stopPropagation();
      close();
      return;
    }
    if (e.key !== "Tab") return;
    // Keep Tab inside the dialog.
    const items = [...dialogRef.current.querySelectorAll(FOCUSABLE)];
    if (items.length === 0) return;
    const first = items[0];
    const last = items[items.length - 1];
    if (e.shiftKey && document.activeElement === first) {
      e.preventDefault();
      last.focus();
    } else if (!e.shiftKey && document.activeElement === last) {
      e.preventDefault();
      first.focus();
    }
  }

  function close() {
    if (created) onCreated?.(created.robot);
    onClose();
  }

  async function handleSubmit(e) {
    e.preventDefault();
    const trimmed = name.trim();
    if (!trimmed) {
      setError("Enter a name for the robot.");
      nameRef.current?.focus();
      return;
    }
    setError("");
    setSubmitting(true);
    try {
      const res = await api.registerRobot({ name: trimmed, model: model.trim() || "Unknown" });
      if (!res?.secret || !res?.robot?.id) throw new Error("Unexpected response from the server.");
      setCreated({ robot: res.robot, secret: res.secret });
    } catch (err) {
      console.error("Failed to add robot:", err);
      setError(
        /HTTP 40[13]/.test(err.message)
          ? "You don't have permission to add robots."
          : "Couldn't add the robot. Check that the name isn't already in use and try again."
      );
    } finally {
      setSubmitting(false);
    }
  }

  async function copy(text) {
    try {
      await navigator.clipboard.writeText(text);
      setCopied(true);
      setTimeout(() => setCopied(false), 2000);
    } catch {
      // Clipboard needs a secure context; the secret is also selectable text.
      setError("Couldn't copy automatically — select the secret and copy it manually.");
    }
  }

  const installCommand = created
    ? `sudo ./install.sh --backend-url ${window.location.origin} --robot-id ${created.robot.id} --secret-file -`
    : "";

  return (
    <div className="fixed inset-0 z-40 flex items-center justify-center p-4" onKeyDown={handleKeyDown}>
      <div className="absolute inset-0 bg-black/40" onClick={() => !locked && !submitting && close()} aria-hidden="true" />
      <div
        ref={dialogRef}
        role="dialog"
        aria-modal="true"
        aria-labelledby={ids.title}
        aria-describedby={ids.desc}
        className="relative z-50 max-h-[90vh] w-full max-w-lg overflow-y-auto rounded-xl bg-white shadow-xl"
      >
        <div className="flex items-start justify-between gap-4 border-b border-gray-200 px-6 py-4">
          <div>
            <h2 id={ids.title} className="text-lg font-semibold text-gray-900">
              {created ? "Robot added" : "Add a robot"}
            </h2>
            <p id={ids.desc} className="mt-1 text-sm text-gray-600">
              {created
                ? "Save this secret now. It is shown once and can't be retrieved later."
                : "Registers the robot and generates its login secret."}
            </p>
          </div>
          {!locked && (
            <button
              type="button"
              onClick={close}
              aria-label="Close"
              className="rounded text-xl leading-none text-gray-400 hover:text-gray-600 focus:outline-none focus:ring-2 focus:ring-indigo-500"
            >
              &times;
            </button>
          )}
        </div>

        {!created ? (
          <form onSubmit={handleSubmit} noValidate className="space-y-4 px-6 py-5">
            <div>
              <label htmlFor={`${ids.title}-name`} className="block text-sm font-medium text-gray-900">
                Name <span aria-hidden="true" className="text-red-600">*</span>
                <span className="sr-only">(required)</span>
              </label>
              <input
                id={`${ids.title}-name`}
                ref={nameRef}
                type="text"
                value={name}
                onChange={(e) => setName(e.target.value)}
                required
                aria-required="true"
                aria-invalid={error ? "true" : undefined}
                aria-describedby={`${ids.title}-name-hint${error ? ` ${ids.error}` : ""}`}
                autoComplete="off"
                className="mt-1 block w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              />
              <p id={`${ids.title}-name-hint`} className="mt-1 text-xs text-gray-500">
                How the robot appears in the fleet, e.g. blossom-07. Must be unique.
              </p>
            </div>

            <div>
              <label htmlFor={`${ids.title}-model`} className="block text-sm font-medium text-gray-900">
                Model
              </label>
              <input
                id={`${ids.title}-model`}
                type="text"
                list={`${ids.title}-models`}
                value={model}
                onChange={(e) => setModel(e.target.value)}
                autoComplete="off"
                className="mt-1 block w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              />
              <datalist id={`${ids.title}-models`}>
                {MODEL_SUGGESTIONS.map((m) => (
                  <option key={m} value={m} />
                ))}
              </datalist>
            </div>

            <p id={ids.error} role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
              {error}
            </p>

            <div className="flex justify-end gap-2 pt-1">
              <button
                type="button"
                onClick={close}
                disabled={submitting}
                className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60"
              >
                Cancel
              </button>
              <button
                type="submit"
                disabled={submitting}
                className="rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1 disabled:opacity-60"
              >
                {submitting ? "Adding…" : "Add robot"}
              </button>
            </div>
          </form>
        ) : (
          <div className="space-y-4 px-6 py-5">
            <dl className="grid grid-cols-[auto,1fr] gap-x-4 gap-y-2 text-sm">
              <dt className="font-medium text-gray-500">Name</dt>
              <dd className="text-gray-900">{created.robot.name}</dd>
              <dt className="font-medium text-gray-500">Robot ID</dt>
              <dd className="break-all font-mono text-gray-900">{created.robot.id}</dd>
            </dl>

            <div>
              <label htmlFor={ids.secret} className="block text-sm font-medium text-gray-900">
                Secret
              </label>
              <div className="mt-1 flex gap-2">
                <input
                  id={ids.secret}
                  type="text"
                  readOnly
                  value={created.secret}
                  onFocus={(e) => e.target.select()}
                  className="min-w-0 flex-1 rounded-md border border-gray-300 bg-gray-50 px-3 py-2 font-mono text-sm"
                />
                <button
                  type="button"
                  onClick={() => copy(created.secret)}
                  className="shrink-0 rounded-md border border-gray-300 px-3 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500"
                >
                  {copied ? "Copied" : "Copy"}
                </button>
              </div>
              <span role="status" className="sr-only">
                {copied ? "Secret copied to clipboard" : ""}
              </span>
            </div>

            <details className="rounded-md bg-gray-50 px-3 py-2 text-sm">
              <summary className="cursor-pointer font-medium text-gray-800 focus:outline-none focus:ring-2 focus:ring-indigo-500">
                Set it up on the robot
              </summary>
              <ol className="mt-2 list-decimal space-y-2 pl-5 text-gray-700">
                <li>
                  For the agent, run this on the robot and paste the secret when prompted:
                  <div className="mt-1 flex gap-2">
                    <code className="block min-w-0 flex-1 break-all rounded bg-white px-2 py-1 font-mono text-xs">
                      {installCommand}
                    </code>
                    <button
                      type="button"
                      onClick={() => copy(installCommand)}
                      className="shrink-0 rounded border border-gray-300 px-2 text-xs font-semibold text-gray-700 hover:bg-white focus:outline-none focus:ring-2 focus:ring-indigo-500"
                    >
                      Copy
                    </button>
                  </div>
                </li>
                <li>
                  For <code className="font-mono text-xs">bloom_node</code>, add{" "}
                  <code className="font-mono text-xs">BLOOM_ROBOT_ID</code> and{" "}
                  <code className="font-mono text-xs">BLOOM_ROBOT_SECRET</code> to{" "}
                  <code className="font-mono text-xs">robot/.env</code> and restart it.
                </li>
              </ol>
            </details>

            <p id={ids.error} role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
              {error}
            </p>

            <label className="flex items-start gap-2 text-sm text-gray-800">
              <input
                type="checkbox"
                checked={saved}
                onChange={(e) => setSaved(e.target.checked)}
                className="mt-0.5 h-4 w-4 rounded border-gray-300 text-indigo-600 focus:ring-indigo-500"
              />
              I've saved the secret. I understand it can't be shown again.
            </label>

            <div className="flex justify-end pt-1">
              <button
                type="button"
                onClick={close}
                disabled={!saved}
                className="rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300"
              >
                Done
              </button>
            </div>
          </div>
        )}
      </div>
    </div>
  );
}
