// RobotTerminal.jsx
// Browser end of the fleet remote shell: an xterm.js terminal wired to the backend's /hubs/shell
// SignalR hub, which pipes it to a PTY on the robot via its agent. One connection = one shell session.
//
// Hub contract (backend/Services/RemoteShell/ShellHub.cs, ShellHubClients.cs):
//   we call   OpenSession(robotId, cols, rows, force) -> { status, sessionId, heldBy, since, message }
//             Input(base64) / Resize(cols, rows) / CloseSession()
//   it calls  Output(base64) / SessionClosed(reason)
// Byte payloads are base64 (SignalR's JSON protocol). Input goes out in small pieces because the
// backend drops a connection that sends a message over its receive limit.

import React, { useCallback, useEffect, useRef, useState } from "react";
import { HubConnectionBuilder, LogLevel } from "@microsoft/signalr";
import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import "@xterm/xterm/css/xterm.css";
import { useApiClient } from "../../context/ApiClientContext";
import { base64ToBytes, encodeInput } from "../../utils/base64";

const CLOSE_REASONS = {
  exited: "The shell exited.",
  admin_closed: "Session closed.",
  admin_disconnected: "Disconnected.",
  taken_over: "Another admin took over this robot's shell.",
  robot_disconnected: "The robot went offline.",
  revoked: "The robot's credentials were revoked.",
  robot_deleted: "The robot was deleted.",
  credentials_rotated: "The robot's credentials were changed. Reconnect to continue.",
  token_expired: "The robot's connection was renewed (token expiry). Reconnect to continue.",
};

export default function RobotTerminal({ robot, onClose }) {
  const api = useApiClient();
  // ApiClientProvider builds a new client every render; keep the terminal and its connection from
  // being torn down and rebuilt whenever that happens.
  const apiRef = useRef(api);
  apiRef.current = api;
  const containerRef = useRef(null);
  const termRef = useRef(null);
  const connectionRef = useRef(null);
  const openRef = useRef(false); // a session is open on the current connection
  // "connecting" | "open" | "busy" | "ended"
  const [status, setStatus] = useState("connecting");
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(null);

  const connect = useCallback(
    async (force = false) => {
      const term = termRef.current;
      if (!term) return;
      connectionRef.current?.stop();

      setStatus("connecting");
      setMessage("");
      setBusy(null);
      openRef.current = false;

      // No automatic reconnect: a dropped connection is a dead session, and silently starting a new
      // shell behind the admin's back would be wrong.
      const connection = new HubConnectionBuilder()
        .withUrl(apiRef.current.shellHubUrl())
        .configureLogging(LogLevel.Warning)
        .build();
      connectionRef.current = connection;

      let ended = false;
      const current = () => connectionRef.current === connection;
      const end = (text) => {
        if (ended || !current()) return;
        ended = true;
        openRef.current = false;
        setStatus("ended");
        setMessage(text);
      };

      connection.on("Output", (base64) => {
        if (current()) term.write(base64ToBytes(base64));
      });
      connection.on("SessionClosed", (reason) => {
        end(CLOSE_REASONS[reason] || `Session ended (${reason}).`);
        connection.stop(); // nothing left on this connection
      });
      connection.onclose(() => end("Connection closed."));

      try {
        await connection.start();
        if (!current()) return connection.stop();
        const result = await connection.invoke("OpenSession", robot.id, term.cols, term.rows, force);
        if (!current()) return;

        if (result.status === "opened") {
          openRef.current = true;
          setStatus("open");
          term.focus();
        } else if (result.status === "busy") {
          ended = true; // the stop below is not a failure
          setStatus("busy");
          setBusy({ heldBy: result.heldBy, since: result.since });
          connection.stop();
        } else {
          end(result.message || "Couldn't open a session.");
          connection.stop();
        }
      } catch (err) {
        console.error("Remote shell connection failed:", err);
        end(
          /\b40[13]\b/.test(err?.message || "")
            ? "You don't have permission to open a shell."
            : "Couldn't connect to the server."
        );
        connection.stop();
      }
    },
    [robot.id]
  );

  // Terminal lifetime: create once, fit to the container, follow its size.
  useEffect(() => {
    const term = new Terminal({
      cursorBlink: true,
      fontFamily: 'ui-monospace, SFMono-Regular, Menlo, Consolas, "Liberation Mono", monospace',
      fontSize: 14,
      scrollback: 5000,
      theme: { background: "#111827" },
    });
    const fit = new FitAddon();
    term.loadAddon(fit);
    term.open(containerRef.current);
    fit.fit();
    termRef.current = term;

    const input = term.onData((data) => {
      const connection = connectionRef.current;
      if (!openRef.current || !connection) return;
      for (const piece of encodeInput(data)) connection.send("Input", piece).catch(() => {});
    });
    const resize = term.onResize(({ cols, rows }) => {
      const connection = connectionRef.current;
      if (openRef.current && connection) connection.send("Resize", cols, rows).catch(() => {});
    });
    const observer = new ResizeObserver(() => fit.fit());
    observer.observe(containerRef.current);

    connect();

    return () => {
      observer.disconnect();
      input.dispose();
      resize.dispose();
      const connection = connectionRef.current;
      connectionRef.current = null;
      if (connection) {
        // Say goodbye so the audit log records an admin close rather than a dropped connection.
        const hangUp = () => connection.stop();
        (openRef.current ? connection.send("CloseSession") : Promise.resolve()).then(hangUp, hangUp);
      }
      term.dispose();
    };
  }, [connect]);

  return (
    <div className="flex h-full flex-col">
      <div className="flex items-center justify-between gap-4 border-b border-gray-200 bg-white px-4 py-3">
        <div className="min-w-0">
          <h2 className="truncate text-base font-semibold text-gray-900">{robot.name}</h2>
          <p className="text-xs text-gray-500">
            {status === "open" && "Connected — shell runs as bloom-ops (sudo). Session is audited."}
            {status === "connecting" && "Connecting…"}
            {(status === "ended" || status === "busy") && "Not connected"}
          </p>
        </div>
        <div className="flex items-center gap-2">
          {status === "ended" && (
            <button
              type="button"
              onClick={() => connect()}
              className="rounded-md border border-gray-300 px-3 py-1.5 text-sm font-semibold text-gray-700 hover:bg-gray-50"
            >
              Reconnect
            </button>
          )}
          <button
            type="button"
            onClick={onClose}
            className="text-xl leading-none text-gray-400 hover:text-gray-600"
            aria-label="Close terminal"
          >
            &times;
          </button>
        </div>
      </div>

      {status === "busy" && busy && (
        <div className="flex items-center justify-between gap-4 bg-amber-50 px-4 py-2 text-sm text-amber-800">
          <span>
            In use by <strong>{busy.heldBy || "another admin"}</strong> since{" "}
            {new Date(busy.since).toLocaleTimeString([], { hour: "2-digit", minute: "2-digit" })}.
          </span>
          <button
            type="button"
            onClick={() => connect(true)}
            className="rounded-md bg-amber-600 px-3 py-1 text-xs font-semibold text-white hover:bg-amber-500"
          >
            Take over
          </button>
        </div>
      )}
      {status === "ended" && message && (
        <div className="bg-gray-100 px-4 py-2 text-sm text-gray-700">{message}</div>
      )}

      <div className="min-h-0 flex-1 bg-gray-900 p-2">
        <div ref={containerRef} className="h-full w-full" />
      </div>
    </div>
  );
}
