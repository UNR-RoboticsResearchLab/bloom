// RobotTerminal.jsx
// Browser end of the fleet remote shell: an xterm.js terminal wired to the backend's /ws/shell
// relay, which pipes it to a PTY on the robot via its agent. One WebSocket = one shell session.
//
// Protocol (backend/Services/RemoteShell/ShellProtocol.cs): terminal bytes are raw binary frames
// both ways; control messages are JSON text frames -- we send {type:"resize"|"close"}, the backend
// sends {type:"opened"|"busy"|"error"|"closed"}.

import React, { useCallback, useEffect, useRef, useState } from "react";
import { Terminal } from "@xterm/xterm";
import { FitAddon } from "@xterm/addon-fit";
import "@xterm/xterm/css/xterm.css";
import { useApiClient } from "../../context/ApiClientContext";

const CLOSE_REASONS = {
  exited: "The shell exited.",
  admin_closed: "Session closed.",
  admin_disconnected: "Disconnected.",
  taken_over: "Another admin took over this robot's shell.",
  robot_disconnected: "The robot went offline.",
  revoked: "The robot's credentials were revoked.",
  token_expired: "The robot's tunnel was renewed (token expiry). Reconnect to continue.",
};

export default function RobotTerminal({ robot, onClose }) {
  const api = useApiClient();
  // ApiClientProvider builds a new client every render; keep the terminal and its socket from
  // being torn down and reconnected whenever that happens.
  const apiRef = useRef(api);
  apiRef.current = api;
  const containerRef = useRef(null);
  const termRef = useRef(null);
  const socketRef = useRef(null);
  // "connecting" | "open" | "busy" | "ended"
  const [status, setStatus] = useState("connecting");
  const [message, setMessage] = useState("");
  const [busy, setBusy] = useState(null);

  const connect = useCallback(
    (force = false) => {
      const term = termRef.current;
      if (!term) return;
      socketRef.current?.close();

      setStatus("connecting");
      setMessage("");
      setBusy(null);

      const ws = new WebSocket(apiRef.current.shellSocketUrl(robot.id, term.cols, term.rows, force));
      ws.binaryType = "arraybuffer";
      socketRef.current = ws;
      let ended = false;
      const end = (text) => {
        if (ended) return;
        ended = true;
        setStatus("ended");
        setMessage(text);
      };

      ws.onmessage = (event) => {
        if (socketRef.current !== ws) return;
        if (event.data instanceof ArrayBuffer) {
          term.write(new Uint8Array(event.data));
          return;
        }
        const msg = JSON.parse(event.data);
        switch (msg.type) {
          case "opened":
            setStatus("open");
            term.focus();
            break;
          case "busy":
            ended = true;
            setStatus("busy");
            setBusy({ heldBy: msg.heldBy, since: msg.since });
            break;
          case "error":
            end(msg.message);
            break;
          case "closed":
            end(CLOSE_REASONS[msg.reason] || `Session ended (${msg.reason}).`);
            break;
          default:
            break;
        }
      };
      ws.onclose = () => {
        if (socketRef.current === ws) end("Connection closed.");
      };
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

    const encoder = new TextEncoder();
    const input = term.onData((data) => {
      const ws = socketRef.current;
      if (ws?.readyState === WebSocket.OPEN) ws.send(encoder.encode(data));
    });
    const resize = term.onResize(({ cols, rows }) => {
      const ws = socketRef.current;
      if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "resize", cols, rows }));
    });
    const observer = new ResizeObserver(() => fit.fit());
    observer.observe(containerRef.current);

    connect();

    return () => {
      observer.disconnect();
      input.dispose();
      resize.dispose();
      const ws = socketRef.current;
      socketRef.current = null;
      if (ws?.readyState === WebSocket.OPEN) ws.send(JSON.stringify({ type: "close" }));
      ws?.close();
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
          </span>The release build is stamped 1.0.3-g965b69c09d, strips to 2.3 MB, and links only libc and OpenSSL. One Release-only warning ((void) doesn't silence warn_unused_result in GCC), fixing it:

￼
￼

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
