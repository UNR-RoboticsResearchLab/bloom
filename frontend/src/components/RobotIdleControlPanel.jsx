import React, { useCallback, useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

const POLL_INTERVAL_MS = 4000;

// Lets a paired user control what the robot does when no lesson is running:
// either free-form spoken conversation, or plain passive/breathing idle while
// it waits for the next lesson to be queued. Disabled while a lesson is active
// — idle mode only applies between lessons.
export function RobotIdleControlPanel({ sessionId }) {
  const api = useApiClient();
  const [mode, setMode] = useState("passive");
  const [activeLessonId, setActiveLessonId] = useState(null);
  const [pendingMode, setPendingMode] = useState(null);
  const [error, setError] = useState("");

  const refresh = useCallback(async () => {
    if (!sessionId) return;
    try {
      const [session, idleMode] = await Promise.all([
        api.getSession(sessionId),
        api.getRobotIdleMode(sessionId),
      ]);
      setActiveLessonId(session?.activeLessonId ?? null);
      setMode(idleMode?.mode ?? "passive");
    } catch (err) {
      console.error("Failed to refresh robot idle mode:", err);
    }
  }, [api, sessionId]);

  useEffect(() => {
    refresh();
    const intervalId = setInterval(refresh, POLL_INTERVAL_MS);
    return () => clearInterval(intervalId);
  }, [refresh]);

  const lessonActive = !!activeLessonId;

  async function handleSetMode(nextMode) {
    const previousMode = mode;
    setError("");
    setPendingMode(nextMode);
    setMode(nextMode); // optimistic
    try {
      await api.setRobotIdleMode(sessionId, nextMode);
    } catch (err) {
      console.error("Failed to set robot idle mode:", err);
      setError("Failed to update idle mode. Please try again.");
      setMode(previousMode);
    } finally {
      setPendingMode(null);
    }
  }

  async function handleStop() {
    const previousMode = mode;
    setError("");
    setPendingMode("passive");
    setMode("passive"); // optimistic
    try {
      await api.stopRobotIdleMode(sessionId);
    } catch (err) {
      console.error("Failed to stop robot idle mode:", err);
      setError("Failed to stop idle mode. Please try again.");
      setMode(previousMode);
    } finally {
      setPendingMode(null);
    }
  }

  return (
    <div className="rounded-lg bg-white p-4 shadow">
      <div className="flex items-center justify-between">
        <h3 className="text-base font-semibold">Robot Idle Behavior</h3>
        {mode === "conversational" && (
          <span className="rounded-full bg-green-100 px-2.5 py-0.5 text-xs font-medium text-green-700">
            Conversing
          </span>
        )}
      </div>

      <p className="mt-1 text-sm text-gray-600">
        {lessonActive
          ? "Available when no lesson is running."
          : "Choose what the robot does while it waits for a lesson."}
      </p>

      <div className="mt-4 flex flex-wrap gap-3">
        <button
          type="button"
          disabled={lessonActive || pendingMode !== null}
          onClick={() => handleSetMode("conversational")}
          className={`rounded-md px-4 py-2 text-sm font-medium shadow-sm disabled:opacity-50 ${
            mode === "conversational"
              ? "bg-indigo-600 text-white hover:bg-indigo-500"
              : "border border-gray-300 bg-white text-gray-700 hover:bg-gray-50"
          }`}
        >
          {pendingMode === "conversational" ? "Starting…" : "Free Conversation"}
        </button>

        <button
          type="button"
          disabled={lessonActive || pendingMode !== null}
          onClick={() => handleSetMode("passive")}
          className={`rounded-md px-4 py-2 text-sm font-medium shadow-sm disabled:opacity-50 ${
            mode === "passive"
              ? "bg-indigo-600 text-white hover:bg-indigo-500"
              : "border border-gray-300 bg-white text-gray-700 hover:bg-gray-50"
          }`}
        >
          {pendingMode === "passive" ? "Switching…" : "Passive Idle"}
        </button>

        {mode === "conversational" && (
          <button
            type="button"
            disabled={lessonActive || pendingMode !== null}
            onClick={handleStop}
            className="rounded-md border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-50 disabled:opacity-50"
          >
            Stop
          </button>
        )}
      </div>

      {error && (
        <div className="mt-3 rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
          {error}
        </div>
      )}
    </div>
  );
}
