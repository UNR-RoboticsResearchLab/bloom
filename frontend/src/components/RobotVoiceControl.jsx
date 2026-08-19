import React, { useCallback, useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

const POLL_INTERVAL_MS = 4000;

// Lets a paired user temporarily override the robot's TTS voice for the
// current session, on top of whatever the paired student has set as their
// persistent default in their RobotProfile. Applies any time, including
// mid-lesson.
export function RobotVoiceControl({ sessionId }) {
  const api = useApiClient();
  const [voice, setVoice] = useState("");
  const [presets, setPresets] = useState([]);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");

  useEffect(() => {
    async function loadPresets() {
      try {
        const data = await api.getRobotVoicePresets();
        setPresets(Array.isArray(data) ? data : []);
      } catch (err) {
        console.error("Failed to load voice presets:", err);
      }
    }
    loadPresets();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  const refresh = useCallback(async () => {
    if (!sessionId) return;
    try {
      const effective = await api.getRobotVoice(sessionId);
      setVoice(effective?.voice ?? "");
    } catch (err) {
      console.error("Failed to refresh robot voice:", err);
    }
  }, [api, sessionId]);

  useEffect(() => {
    refresh();
    const intervalId = setInterval(refresh, POLL_INTERVAL_MS);
    return () => clearInterval(intervalId);
  }, [refresh]);

  async function handleChange(nextVoice) {
    const previousVoice = voice;
    setError("");
    setPending(true);
    setVoice(nextVoice); // optimistic
    try {
      await api.setRobotVoice(sessionId, nextVoice);
    } catch (err) {
      console.error("Failed to set robot voice:", err);
      setError("Failed to update voice. Please try again.");
      setVoice(previousVoice);
    } finally {
      setPending(false);
    }
  }

  async function handleReset() {
    setError("");
    setPending(true);
    try {
      await api.resetRobotVoice(sessionId);
      await refresh();
    } catch (err) {
      console.error("Failed to reset robot voice:", err);
      setError("Failed to reset voice. Please try again.");
    } finally {
      setPending(false);
    }
  }

  return (
    <div className="rounded-lg bg-white p-4 shadow">
      <h3 className="text-base font-semibold">Robot Voice</h3>
      <p className="mt-1 text-sm text-gray-600">
        Override the robot's voice for this session only.
      </p>

      <div className="mt-4 flex flex-wrap items-center gap-3">
        <select
          value={voice}
          disabled={pending}
          onChange={(e) => handleChange(e.target.value)}
          className="rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200 disabled:opacity-50"
        >
          {presets.map((preset) => (
            <option key={preset.id} value={preset.id}>
              {preset.label}
            </option>
          ))}
        </select>

        <button
          type="button"
          disabled={pending}
          onClick={handleReset}
          className="rounded-md border border-gray-300 bg-white px-4 py-2 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-50 disabled:opacity-50"
        >
          Reset to My Default
        </button>
      </div>

      {error && (
        <div className="mt-3 rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
          {error}
        </div>
      )}
    </div>
  );
}
