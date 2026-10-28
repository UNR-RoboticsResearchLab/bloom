// AddRobotDialog.jsx
// Admin flow for adding a robot: name + model -> POST /api/robot/register -> the robot's login
// secret, which the backend shows exactly once. The second step can't be dismissed (no backdrop
// click, Escape or close button) until the admin confirms they've saved it.

import React, { useId, useRef, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import SecretReveal from "./SecretReveal";
import { apiErrorStatus } from "../../utils/provisioning";

const MODEL_SUGGESTIONS = ["Blossom"];
const inputClass =
  "mt-1 block w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200";

export default function AddRobotDialog({ onClose, onCreated }) {
  const api = useApiClient();
  const id = useId();
  const nameRef = useRef(null);

  const [name, setName] = useState("");
  const [model, setModel] = useState(MODEL_SUGGESTIONS[0]);
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState("");
  const [created, setCreated] = useState(null); // { robot, secret }
  const [saved, setSaved] = useState(false);

  const locked = !!created && !saved; // closing now would lose the secret forever

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
        [401, 403].includes(apiErrorStatus(err))
          ? "You don't have permission to add robots."
          : "Couldn't add the robot. Check that the name isn't already in use and try again."
      );
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={close} dismissible={!locked && !submitting}>
      <div className="flex items-start justify-between gap-4 border-b border-gray-200 px-6 py-4">
        <div>
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">
            {created ? "Robot added" : "Add a robot"}
          </h2>
          <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">
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
            <label htmlFor={`${id}-name`} className="block text-sm font-medium text-gray-900">
              Name <span aria-hidden="true" className="text-red-600">*</span>
              <span className="sr-only">(required)</span>
            </label>
            <input
              id={`${id}-name`}
              ref={nameRef}
              data-autofocus
              type="text"
              value={name}
              onChange={(e) => setName(e.target.value)}
              required
              aria-required="true"
              aria-invalid={error ? "true" : undefined}
              aria-describedby={`${id}-name-hint${error ? ` ${id}-error` : ""}`}
              autoComplete="off"
              className={inputClass}
            />
            <p id={`${id}-name-hint`} className="mt-1 text-xs text-gray-500">
              How the robot appears in the fleet, e.g. blossom-07. Must be unique.
            </p>
          </div>

          <div>
            <label htmlFor={`${id}-model`} className="block text-sm font-medium text-gray-900">
              Model
            </label>
            <input
              id={`${id}-model`}
              type="text"
              list={`${id}-models`}
              value={model}
              onChange={(e) => setModel(e.target.value)}
              autoComplete="off"
              className={inputClass}
            />
            <datalist id={`${id}-models`}>
              {MODEL_SUGGESTIONS.map((m) => (
                <option key={m} value={m} />
              ))}
            </datalist>
          </div>

          <p id={`${id}-error`} role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
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
          <SecretReveal robot={created.robot} secret={created.secret} saved={saved} onSavedChange={setSaved} />
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
    </Modal>
  );
}
