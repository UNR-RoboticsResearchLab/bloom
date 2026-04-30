import React, { useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

export function PairRobotCard({ onCancel, onPaired }) {
  const [robotCode, setRobotCode] = useState("");
  const [sessionId, setSessionId] = useState(() => {
    return localStorage.getItem("pairedSessionId") || "";
  });
  const [error, setError] = useState("");

  const api = useApiClient();

  async function handleSubmit(e) {
    e.preventDefault();
    if (!robotCode.trim()) return;

    try {
      setError("");

      const res = await api.getSessionIdFromRobotCode(robotCode);
      const returnedSessionId = res?.id;

      if (returnedSessionId) {
        setSessionId(returnedSessionId);
        localStorage.setItem("pairedSessionId", returnedSessionId);
        console.log("Session ID:", returnedSessionId);

        if (onPaired) {
          onPaired(returnedSessionId);
        }
      } else {
        setError("No session ID returned from server.");
        console.error("Invalid response:", res);
      }
    } catch (err) {
      console.error("Failed to pair robot:", err);
      setError("Failed to pair robot.");
    }
  }

  return (
    <div className="w-full max-w-xl rounded-lg border border-gray-300 bg-white shadow-sm">
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">Pair Robot</p>
        <p className="mt-1 text-sm text-gray-500">Pair a robot to a session.</p>
      </div>

      <form onSubmit={handleSubmit} className="px-6 py-5">
        <div className="space-y-4">
          <div>
            <label className="text-sm font-medium text-gray-700">Robot Code</label>
            <input
              value={robotCode}
              onChange={(e) => setRobotCode(e.target.value)}
              className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              placeholder="Enter robot code"
            />
          </div>

          {sessionId && (
            <div className="rounded-md border border-green-200 bg-green-50 px-3 py-2 text-sm text-green-700">
              Session ID: {sessionId}
            </div>
          )}

          {error && (
            <div className="rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
              {error}
            </div>
          )}
        </div>

        <div className="mt-6 border-t border-gray-200 pt-5">
          <div className="flex justify-end gap-3">
            <button
              type="button"
              onClick={onCancel}
              className="rounded-md border border-gray-300 bg-red-500 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-red-400"
            >
              Cancel
            </button>

            <button
              type="submit"
              className="rounded-md border border-gray-900 bg-indigo-600 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-indigo-400"
            >
              Pair Robot
            </button>
          </div>
        </div>
      </form>
    </div>
  );
}