import React, { useState } from "react";
import { useRobotPairing } from "../context/RobotPairingContext";

export function PairRobotCard({ onCancel, onPaired, onUnpaired }) {
  const { sessionId, robotCode: storedCode, isPaired, pair, unpair } = useRobotPairing();
  const [robotCode, setRobotCode] = useState("");
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  async function handleSubmit(e) {
    e.preventDefault();
    if (!robotCode.trim()) return;

    try {
      setIsLoading(true);
      setError("");

      // joinSessionByCode (not getSessionIdFromRobotCode) -- this needs to
      // actually claim the session (set its UserId) so the session reflects
      // as paired for anything polling it, e.g. the robot's own pairing-code
      // display, which otherwise never sees userId set and never clears.
      const res = await api.joinSessionByCode(robotCode);
      const returnedSessionId = res?.id;

      if (returnedSessionId) {
        localStorage.setItem("pairedSessionId", returnedSessionId);
        localStorage.setItem("pairedRobotCode", robotCode);
        if (onPaired) onPaired(returnedSessionId);
      } else {
        setError("No session ID returned from server.");
      }
      const returnedSessionId = await pair(robotCode);
      if (onPaired) onPaired(returnedSessionId);
    } catch (err) {
      console.error("Failed to pair robot:", err);
      setError("Failed to pair robot. Check the code and try again.");
    } finally {
      setIsLoading(false);
    }
  }

  async function handleUnpair() {
    try {
      setIsLoading(true);
      setError("");
      await unpair();
      if (onUnpaired) onUnpaired();
    } catch (err) {
      console.error("Failed to unpair robot:", err);
      setError("Failed to unpair robot.");
    } finally {
      setIsLoading(false);
    }
  }

  return (
    <div className="w-full max-w-xl rounded-lg border border-gray-300 bg-white shadow-sm">
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">
          {isPaired ? "Robot Connected" : "Pair Robot"}
        </p>
        <p className="mt-1 text-sm text-gray-500">
          {isPaired
            ? "Your robot is paired and will stay connected as you move around the app."
            : "Pair a robot to a session."}
        </p>
      </div>

      <div className="px-6 py-5">
        <div className="space-y-4">
          {isPaired ? (
            <>
              {storedCode && (
                <div className="rounded-md border border-gray-200 bg-gray-50 px-3 py-2 text-sm text-gray-700">
                  <span className="font-medium">Pairing Code:</span> {storedCode}
                </div>
              )}
              {sessionId && (
                <div className="rounded-md border border-green-200 bg-green-50 px-3 py-2 text-sm text-green-700">
                  <span className="font-medium">Session ID:</span> {sessionId}
                </div>
              )}
            </>
          ) : (
            <form onSubmit={handleSubmit}>
              <div>
                <label className="text-sm font-medium text-gray-700">Robot Code</label>
                <input
                  value={robotCode}
                  onChange={(e) => setRobotCode(e.target.value)}
                  className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                  placeholder="Enter robot code"
                />
              </div>
              {error && (
                <div className="mt-3 rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
                  {error}
                </div>
              )}
              <div className="mt-6 border-t border-gray-200 pt-5 flex justify-end gap-3">
                <button
                  type="button"
                  onClick={onCancel}
                  className="rounded-md border border-gray-300 bg-red-500 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-red-400"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={isLoading}
                  className="rounded-md border border-gray-900 bg-indigo-600 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-indigo-400 disabled:opacity-60"
                >
                  {isLoading ? "Pairing..." : "Pair Robot"}
                </button>
              </div>
            </form>
          )}

          {isPaired && error && (
            <div className="rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
              {error}
            </div>
          )}
        </div>

        {isPaired && (
          <div className="mt-6 border-t border-gray-200 pt-5 flex justify-end gap-3">
            <button
              type="button"
              onClick={onCancel}
              className="rounded-md border border-gray-300 bg-gray-100 px-4 py-2 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-200"
            >
              Close
            </button>
            <button
              type="button"
              onClick={handleUnpair}
              disabled={isLoading}
              className="rounded-md bg-red-500 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-red-400 disabled:opacity-60"
            >
              {isLoading ? "Unpairing..." : "Unpair"}
            </button>
          </div>
        )}
      </div>
    </div>
  );
}
