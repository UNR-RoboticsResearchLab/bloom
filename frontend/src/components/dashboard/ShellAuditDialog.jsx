// ShellAuditDialog.jsx
// Remote shell audit log for one robot (GET /api/robot/{id}/shell-sessions, newest first):
// who opened a shell, when, for how long, and why it ended.

import React, { useEffect, useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";

// Labels for the backend's ShellEndReason values; unknown ones are shown as sent.
const END_REASONS = {
  exited: "Shell exited",
  admin_closed: "Closed by admin",
  admin_disconnected: "Admin disconnected",
  taken_over: "Taken over by another admin",
  robot_disconnected: "Robot disconnected",
  revoked: "Robot revoked",
  token_expired: "Token expired",
  server_shutdown: "Server shut down",
  credentials_rotated: "Credentials rotated",
  robot_deleted: "Robot deleted",
};

function formatDuration(start, end) {
  const seconds = Math.max(0, Math.round((new Date(end) - new Date(start)) / 1000));
  if (seconds < 60) return `${seconds}s`;
  if (seconds < 3600) return `${Math.floor(seconds / 60)}m ${seconds % 60}s`;
  return `${Math.floor(seconds / 3600)}h ${Math.floor((seconds % 3600) / 60)}m`;
}

export default function ShellAuditDialog({ robot, onClose }) {
  const api = useApiClient();
  const id = useId();
  const [sessions, setSessions] = useState(null);
  const [error, setError] = useState("");

  useEffect(() => {
    let mounted = true;
    api
      .getShellSessions(robot.id)
      .then((data) => mounted && setSessions(data))
      .catch((err) => {
        console.error("Failed to load shell sessions:", err);
        if (mounted) setError("Couldn't load the shell history.");
      });
    return () => {
      mounted = false;
    };
  }, [api, robot.id]);

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={onClose}>
      <div className="border-b border-gray-200 px-6 py-4">
        <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">
          Shell history for {robot.name}
        </h2>
        <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">
          Remote shell sessions, newest first.
        </p>
      </div>

      <div className="px-6 py-5">
        {error ? (
          <p role="alert" className="text-sm text-red-600">{error}</p>
        ) : sessions === null ? (
          <p className="text-sm text-gray-500">Loading…</p>
        ) : sessions.length === 0 ? (
          <p className="text-sm text-gray-500">No shell sessions yet.</p>
        ) : (
          <div className="max-h-[50vh] overflow-auto rounded-lg border border-gray-200">
            <table className="w-full text-left text-sm">
              <thead className="sticky top-0 bg-gray-50 text-xs uppercase tracking-wide text-gray-500">
                <tr>
                  <th className="px-3 py-2 font-medium">Admin</th>
                  <th className="px-3 py-2 font-medium">Started</th>
                  <th className="px-3 py-2 font-medium">Duration</th>
                  <th className="px-3 py-2 font-medium">Ended</th>
                </tr>
              </thead>
              <tbody className="divide-y divide-gray-100">
                {sessions.map((s) => (
                  <tr key={s.id}>
                    <td className="px-3 py-2 text-gray-900">{s.adminEmail || s.adminUserId}</td>
                    <td className="whitespace-nowrap px-3 py-2 text-gray-700">
                      {new Date(s.startedAt).toLocaleString()}
                    </td>
                    <td className="whitespace-nowrap px-3 py-2 text-gray-700">
                      {s.endedAt ? formatDuration(s.startedAt, s.endedAt) : "—"}
                    </td>
                    <td className="px-3 py-2 text-gray-700">
                      {s.endedAt ? END_REASONS[s.endReason] || s.endReason || "Ended" : "Open now"}
                    </td>
                  </tr>
                ))}
              </tbody>
            </table>
          </div>
        )}

        <div className="flex justify-end pt-4">
          <button
            type="button"
            onClick={onClose}
            data-autofocus
            className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500"
          >
            Close
          </button>
        </div>
      </div>
    </Modal>
  );
}
