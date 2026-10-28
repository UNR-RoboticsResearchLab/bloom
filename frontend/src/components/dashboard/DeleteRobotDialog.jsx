// DeleteRobotDialog.jsx
// Confirm-by-typing delete for a robot. Deleting is permanent and cuts the robot off, so the button
// stays disabled until the robot's name is typed exactly. Cancel is the default focus.

import React, { useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import { apiErrorStatus } from "../../utils/provisioning";

export default function DeleteRobotDialog({ robot, onClose, onDeleted }) {
  const api = useApiClient();
  const id = useId();
  const [typed, setTyped] = useState("");
  const [deleting, setDeleting] = useState(false);
  const [error, setError] = useState("");

  const matches = typed.trim() === robot.name;

  async function handleSubmit(e) {
    e.preventDefault();
    if (!matches) return;
    setError("");
    setDeleting(true);
    try {
      await api.deleteRobot(robot.id);
      onDeleted?.(robot);
      onClose();
    } catch (err) {
      console.error("Failed to delete robot:", err);
      setError(
        [401, 403].includes(apiErrorStatus(err))
          ? "You don't have permission to delete robots."
          : "Couldn't delete the robot. It may already be gone; refresh and try again."
      );
      setDeleting(false);
    }
  }

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={onClose} dismissible={!deleting}>
      <form onSubmit={handleSubmit}>
        <div className="border-b border-gray-200 px-6 py-4">
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">
            Delete {robot.name}?
          </h2>
          <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">
            This permanently removes the robot from the fleet.
          </p>
        </div>

        <div className="space-y-4 px-6 py-5">
          <ul className="list-disc space-y-1 pl-5 text-sm text-gray-700">
            <li>The robot can no longer log in, and its credentials stop working.</li>
            {robot.online && <li>It is online now: its connection and any open shell are closed immediately.</li>}
            <li>Its shell session history is kept in the audit log.</li>
            <li>To use it again, you'd add it as a new robot and provision it from scratch.</li>
          </ul>

          <div>
            <label htmlFor={`${id}-confirm`} className="block text-sm font-medium text-gray-900">
              Type <span className="font-mono">{robot.name}</span> to confirm
            </label>
            <input
              id={`${id}-confirm`}
              type="text"
              value={typed}
              onChange={(e) => setTyped(e.target.value)}
              autoComplete="off"
              spellCheck={false}
              className="mt-1 block w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-red-500 focus:outline-none focus:ring-2 focus:ring-red-200"
            />
          </div>

          <p role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
            {error}
          </p>

          <div className="flex justify-end gap-2 pt-1">
            <button
              type="button"
              onClick={onClose}
              disabled={deleting}
              data-autofocus
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60"
            >
              Cancel
            </button>
            <button
              type="submit"
              disabled={!matches || deleting}
              className="rounded-md bg-red-600 px-4 py-2 text-sm font-semibold text-white hover:bg-red-500 focus:outline-none focus:ring-2 focus:ring-red-500 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300"
            >
              {deleting ? "Deleting…" : "Delete robot"}
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
}
