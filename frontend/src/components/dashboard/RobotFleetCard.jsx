// RobotFleetCard.jsx
// One robot in the admin fleet-monitoring grid: model, online status and the
// agent version it last reported, a Connect button for the remote shell, and a
// channel/version picker for OTA updates.
//
// Presentational only — the parent owns data loading, the terminal and the
// actual OTA call. `robot` is a RobotFleetDto (GET /api/robot/fleet).
// `onConnect(robot)` opens a shell. `onUpdate(robot, { channel, version })`
// returns a promise; without it (until OTA lands) the update button stays
// disabled.

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
function relativeTime(iso) {
  if (!iso) return "never";
  const seconds = Math.round((Date.now() - new Date(iso).getTime()) / 1000);
  if (seconds < 60) return "just now";
  if (seconds < 3600) return `${Math.floor(seconds / 60)} min ago`;
  if (seconds < 86400) return `${Math.floor(seconds / 3600)} h ago`;
  return new Date(iso).toLocaleDateString();
}

export default function RobotFleetCard({ robot, releases = {}, onUpdate, onConnect, onCredentials, onDelete }) {
  // The agent's self-reported SemVer; bloom_node's firmwareVersion for robots without an agent yet.
  const currentVersion = robot.agentVersion || robot.firmwareVersion || null;
  const model = robot.model || "Unknown model";
  const name = robot.name;
  const online = !!robot.online;

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
        <span
          className={[
            "inline-flex shrink-0 items-center gap-1.5 rounded-full px-2.5 py-0.5 text-xs font-medium",
            online ? "bg-green-100 text-green-700" : "bg-gray-100 text-gray-600",
          ].join(" ")}
          title={online ? "Agent connected" : `Last seen ${relativeTime(robot.lastSeenAt)}`}
        >
          <span className={["h-2 w-2 rounded-full", online ? "bg-green-500" : "bg-gray-400"].join(" ")} />
          {online ? "Online" : "Offline"}
        </span>
      </div>

      <div className="flex flex-col gap-1 rounded-lg bg-gray-50 px-3 py-2">
        <div className="flex items-center justify-between">
          <span className="text-xs font-medium uppercase tracking-wide text-gray-500">Running</span>
          <span className="font-mono text-sm text-gray-900" title={robot.agentCommit || undefined}>
            {currentVersion || "—"}
          </span>
        </div>
        {!online && (
          <span className="text-right text-xs text-gray-500">Last seen {relativeTime(robot.lastSeenAt)}</span>
        )}
        {robot.provisioned === false && (
          <span className="text-right text-xs text-amber-700">No credentials — can't log in</span>
        )}
      </div>

      {onConnect && (
        <div className="flex flex-col gap-1">
          <button
            type="button"
            onClick={() => onConnect(robot)}
            disabled={!online}
            className="w-full rounded-md border border-gray-300 bg-white px-4 py-2 text-sm font-semibold text-gray-800 transition hover:bg-gray-50 disabled:cursor-not-allowed disabled:text-gray-400"
          >
            {robot.shellHeldBy ? "Connect (in use)" : "Connect"}
          </button>
          {robot.shellHeldBy && (
            <span className="text-xs text-gray-500">
              Shell open by {robot.shellHeldBy}
              {robot.shellSince ? ` since ${new Date(robot.shellSince).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}` : ""}
            </span>
          )}
        </div>
      )}

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

      {(onCredentials || onDelete) && (
        <div className="flex gap-2">
          {onCredentials && (
            <button
              type="button"
              onClick={() => onCredentials(robot)}
              aria-label={`Credentials for ${name}`}
              className="flex-1 rounded-md border border-gray-300 bg-white px-3 py-2 text-sm font-semibold text-gray-700 transition hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500"
            >
              Credentials
            </button>
          )}
          {onDelete && (
            <button
              type="button"
              onClick={() => onDelete(robot)}
              aria-label={`Delete ${name}`}
              className="rounded-md border border-red-200 bg-white px-3 py-2 text-sm font-semibold text-red-700 transition hover:bg-red-50 focus:outline-none focus:ring-2 focus:ring-red-500"
            >
              Delete
            </button>
          )}
        </div>
      )}

      {!onUpdate && <p className="-mt-2 text-xs text-gray-500">OTA updates aren't available yet.</p>}

      {error && <p className="text-sm text-red-600">{error}</p>}
    </Card>
  );
}
