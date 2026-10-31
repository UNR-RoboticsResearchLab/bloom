// RevokeRobotDialog.jsx
// Confirm dialog for revoking a robot's credentials. Unlike delete, the robot stays in the fleet and
// an admin can bring it back by issuing new credentials. Cancel is the default focus.

import React, { useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import { apiErrorStatus } from "../../utils/provisioning";

export default function RevokeRobotDialog({ robot, onClose, onRevoked }) {
  const api = useApiClient();
  const id = useId();
  const [revoking, setRevoking] = useState(false);
  const [error, setError] = useState("");

  async function handleSubmit(e) {
    e.preventDefault();
    setError("");
    setRevoking(true);
    try {
      await api.revokeRobot(robot.id);
      onRevoked?.(robot);
      onClose();
    } catch (err) {
      console.error("Failed to revoke robot:", err);
      setError(
        [401, 403].includes(apiErrorStatus(err))
          ? "You don't have permission to revoke robots."
          : "Couldn't revoke the robot. Refresh and try again."
      );
      setRevoking(false);
    }
  }

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={onClose} dismissible={!revoking}>
      <form onSubmit={handleSubmit}>
        <div className="border-b border-gray-200 px-6 py-4">
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">
            Revoke {robot.name}?
          </h2>
          <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">
            This cuts the robot off until you issue it new credentials.
          </p>
        </div>

        <div className="space-y-4 px-6 py-5">
          <ul className="list-disc space-y-1 pl-5 text-sm text-gray-700">
            <li>Its secret is removed and all of its tokens stop working.</li>
            {robot.online && <li>It is online now: its connection and any open shell are closed immediately.</li>}
            <li>It stays in the fleet. To bring it back, rotate its credentials and provision it again.</li>
          </ul>

          <p role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
            {error}
          </p>

          <div className="flex justify-end gap-2 pt-1">
            <button
              type="button"
              onClick={onClose}
              disabled={revoking}
              data-autofocus
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60"
            >
              Cancel
            </button>
            <button
              type="submit"
              disabled={revoking}
              className="rounded-md bg-red-600 px-4 py-2 text-sm font-semibold text-white hover:bg-red-500 focus:outline-none focus:ring-2 focus:ring-red-500 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300"
            >
              {revoking ? "Revoking…" : "Revoke robot"}
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
}
