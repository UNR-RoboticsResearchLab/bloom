// RobotFleetCard.jsx
// One robot in the admin fleet-monitoring grid: model, the software version it
// last reported, and a channel/version picker that pushes an OTA update.
//
// Presentational only — the parent owns data loading and the actual OTA call.
// Pass `onUpdate(robot, { channel, version })` returning a promise; the card
// handles its own pending/error/confirm state around it.

import React, { useEffect, useMemo, useState } from "react";
import Card from "../ui/Card";
import Badge from "../ui/Badge";

export const RELEASE_CHANNELS = [
  { id: "prod", label: "Prod", color: "bg-green-100 text-green-700" },
  { id: "canary", label: "Canary", color: "bg-yellow-100 text-yellow-700" },
  { id: "dev", label: "Dev", color: "bg-purple-100 text-purple-700" },
];

function RobotGlyph({ className = "" }) {
  return (
    <svg viewBox="0 0 24 24" fill="none" stroke="currentColor" strokeWidth={1.5} className={className} aria-hidden="true">
      <path strokeLinecap="round" d="M12 3v3" />
      <circle cx="12" cy="2.5" r="1" />
      <rect x="4" y="6" width="16" height="12" rx="4" />
      <circle cx="9" cy="12" r="1.5" fill="currentColor" />
      <circle cx="15" cy="12" r="1.5" fill="currentColor" />
      <path strokeLinecap="round" d="M2 11v3M22 11v3M9 21h6M12 18v3" />
    </svg>
  );
}

// Per-model icon overrides keyed by Robot.Model (case-insensitive). Anything
// not listed falls back to the generic glyph.
const MODEL_ICONS = {};

function ModelIcon({ model }) {
  const Icon = MODEL_ICONS[(model || "").toLowerCase()] || RobotGlyph;
  return (
    <div className="flex h-12 w-12 shrink-0 items-center justify-center rounded-lg bg-indigo-50 text-indigo-600">
      <Icon className="h-7 w-7" />
    </div>
  );
}

// releases: { prod: ["1.4.2", ...], canary: [...], dev: [...] }, newest first.
export default function RobotFleetCard({ robot, releases = {}, onUpdate }) {
  const currentVersion = robot.firmwareVersion || robot.FirmwareVersion || null;
  const model = robot.model || robot.Model || "Unknown model";
  const name = robot.name || robot.Name;

  const [channel, setChannel] = useState("prod");
  const versions = useMemo(() => releases[channel] || [], [releases, channel]);
  const [version, setVersion] = useState(versions[0] || "");
  const [confirming, setConfirming] = useState(false);
  const [pending, setPending] = useState(false);
  const [error, setError] = useState("");

  // Default to the newest build whenever the channel (or its release list) changes.
  useEffect(() => {
    setVersion(versions[0] || "");
    setConfirming(false);
  }, [versions]);

  const isCurrent = version && version === currentVersion;
  const canUpdate = !!onUpdate && !!version && !isCurrent && !pending;
  const channelMeta = RELEASE_CHANNELS.find((c) => c.id === channel);

  async function handleUpdate() {
    if (!confirming) {
      setConfirming(true);
      return;
    }
    setError("");
    setPending(true);
    try {
      await onUpdate(robot, { channel, version });
      setConfirming(false);
    } catch (err) {
      console.error("OTA update failed:", err);
      setError("Failed to push update. Please try again.");
    } finally {
      setPending(false);
    }
  }

  return (
    <Card className="flex flex-col gap-4">
      <div className="flex items-start gap-3">
        <ModelIcon model={model} />
        <div className="min-w-0 flex-1">
          <h3 className="truncate text-base font-semibold text-gray-900">{name}</h3>
          <p className="truncate text-sm text-gray-500">{model}</p>
        </div>
      </div>

      <div className="flex items-center justify-between rounded-lg bg-gray-50 px-3 py-2">
        <span className="text-xs font-medium uppercase tracking-wide text-gray-500">Running</span>
        <span className="font-mono text-sm text-gray-900">{currentVersion || "—"}</span>
      </div>

      <div className="flex flex-col gap-2">
        <div className="flex gap-1 rounded-lg bg-gray-100 p-1" role="radiogroup" aria-label="Release channel">
          {RELEASE_CHANNELS.map((c) => (
            <button
              key={c.id}
              type="button"
              role="radio"
              aria-checked={channel === c.id}
              onClick={() => setChannel(c.id)}
              disabled={pending}
              className={[
                "flex-1 rounded-md px-2 py-1 text-xs font-semibold transition",
                channel === c.id ? "bg-white text-gray-900 shadow-sm" : "text-gray-500 hover:text-gray-700",
              ].join(" ")}
            >
              {c.label}
            </button>
          ))}
        </div>

        <select
          value={version}
          onChange={(e) => {
            setVersion(e.target.value);
            setConfirming(false);
          }}
          disabled={pending || versions.length === 0}
          aria-label="Target version"
          className="w-full rounded-md border border-gray-300 bg-white px-3 py-2 font-mono text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200 disabled:bg-gray-100"
        >
          {versions.length === 0 ? (
            <option value="">No {channelMeta.label.toLowerCase()} builds</option>
          ) : (
            versions.map((v) => (
              <option key={v} value={v}>
                {v}
                {v === currentVersion ? " (current)" : ""}
              </option>
            ))
          )}
        </select>
      </div>

      <button
        type="button"
        onClick={handleUpdate}
        disabled={!canUpdate}
        className={[
          "w-full rounded-md px-4 py-2 text-sm font-semibold text-white transition disabled:cursor-not-allowed disabled:bg-gray-300",
          confirming ? "bg-amber-600 hover:bg-amber-500" : "bg-indigo-600 hover:bg-indigo-500",
        ].join(" ")}
      >
        {pending
          ? "Pushing update…"
          : isCurrent
          ? "Already on this version"
          : confirming
          ? `Confirm push ${version}`
          : version
          ? `Update to ${version}`
          : "Update"}
      </button>

      {confirming && !pending && (
        <div className="-mt-2 flex items-center justify-between text-xs text-gray-500">
          <span>
            Pushes <Badge color={channelMeta.color}>{channelMeta.label}</Badge> build to this robot.
          </span>
          <button type="button" onClick={() => setConfirming(false)} className="text-gray-500 hover:text-gray-700">
            Cancel
          </button>
        </div>
      )}

      {error && <p className="text-sm text-red-600">{error}</p>}
    </Card>
  );
}
