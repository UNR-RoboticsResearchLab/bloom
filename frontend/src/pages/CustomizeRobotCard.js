import React, { useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";

export function CustomizeRobotCard({ profile, onCancel, onSaved }) {
  const [nickname, setNickname] = useState(profile?.nickname || "");
  const [personalityTrait, setPersonalityTrait] = useState(profile?.personalityTrait || "");
  const [catchphrase, setCatchphrase] = useState(profile?.catchphrase || "");
  const [colorTheme, setColorTheme] = useState(profile?.colorTheme || "#6366f1");
  const [presets, setPresets] = useState([]);
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  const api = useApiClient();

  useEffect(() => {
    async function loadPresets() {
      try {
        const data = await api.getRobotPersonalityPresets();
        const list = Array.isArray(data) ? data : [];
        setPresets(list);
        setPersonalityTrait((current) => current || list[0] || "");
      } catch (err) {
        console.error("Failed to load personality presets:", err);
      }
    }
    loadPresets();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  async function handleSubmit(e) {
    e.preventDefault();
    if (!nickname.trim() || !personalityTrait) return;

    try {
      setIsLoading(true);
      setError("");

      const saved = await api.updateMyRobotProfile({
        nickname: nickname.trim(),
        personalityTrait,
        catchphrase: catchphrase.trim() || null,
        colorTheme,
      });

      if (onSaved) onSaved(saved?.profile ?? saved);
    } catch (err) {
      console.error("Failed to save robot profile:", err);
      setError("Failed to save your robot's profile. Please try again.");
    } finally {
      setIsLoading(false);
    }
  }

  return (
    <div className="w-full max-w-xl rounded-lg border border-gray-300 bg-white shadow-sm">
      <div className="border-b border-gray-300 px-6 py-4">
        <p className="text-lg font-semibold text-gray-900">Customize Your Robot</p>
        <p className="mt-1 text-sm text-gray-500">
          Give your robot a name and personality that's all its own.
        </p>
      </div>

      <div className="px-6 py-5">
        <form onSubmit={handleSubmit}>
          <div className="space-y-4">
            <div>
              <label className="text-sm font-medium text-gray-700">Robot Nickname</label>
              <input
                value={nickname}
                onChange={(e) => setNickname(e.target.value)}
                maxLength={50}
                className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                placeholder="e.g. Sparky"
              />
            </div>

            <div>
              <label className="text-sm font-medium text-gray-700">Personality</label>
              <select
                value={personalityTrait}
                onChange={(e) => setPersonalityTrait(e.target.value)}
                className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              >
                {presets.map((preset) => (
                  <option key={preset} value={preset}>
                    {preset}
                  </option>
                ))}
              </select>
            </div>

            <div>
              <label className="text-sm font-medium text-gray-700">Catchphrase (optional)</label>
              <input
                value={catchphrase}
                onChange={(e) => setCatchphrase(e.target.value)}
                maxLength={200}
                className="mt-2 w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                placeholder="e.g. Let's do this!"
              />
            </div>

            <div>
              <label className="text-sm font-medium text-gray-700">Color</label>
              <input
                type="color"
                value={colorTheme}
                onChange={(e) => setColorTheme(e.target.value)}
                className="mt-2 h-10 w-16 cursor-pointer rounded-md border border-gray-300 shadow-sm"
              />
            </div>
          </div>

          {error && (
            <div className="mt-4 rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
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
              disabled={isLoading || !nickname.trim() || !personalityTrait}
              className="rounded-md border border-gray-900 bg-indigo-600 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-indigo-400 disabled:opacity-60"
            >
              {isLoading ? "Saving..." : "Save"}
            </button>
          </div>
        </form>
      </div>
    </div>
  );
}
