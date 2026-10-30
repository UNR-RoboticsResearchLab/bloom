import React from "react";
import { act, render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";

import RobotTerminal from "../RobotTerminal";
import { useApiClient } from "../../../context/ApiClientContext";
import { base64ToBytes, bytesToBase64 } from "../../../utils/base64";

jest.mock("../../../context/ApiClientContext", () => ({ useApiClient: jest.fn() }));

// A SignalR connection that records what the component does with it and lets a test play the server.
const mockConnections = [];
// Lets a test script how each new connection behaves (its index is 0 for the first one created).
let mockOnConnection = null;
jest.mock("@microsoft/signalr", () => {
  class FakeConnection {
    constructor(url) {
      this.url = url;
      this.handlers = {};
      this.closeHandlers = [];
      this.invokeResult = { status: "opened", sessionId: "s1" };
      this.start = jest.fn(async () => {});
      this.invoke = jest.fn(async () => this.invokeResult);
      this.send = jest.fn(async () => {});
      this.stop = jest.fn(async () => this.closeHandlers.forEach((h) => h()));
      mockConnections.push(this);
      if (mockOnConnection) mockOnConnection(this, mockConnections.length - 1);
    }
    on(name, fn) { this.handlers[name] = fn; }
    onclose(fn) { this.closeHandlers.push(fn); }
    emit(name, ...args) { this.handlers[name](...args); }
  }
  class HubConnectionBuilder {
    withUrl(url) { this.url = url; return this; }
    configureLogging() { return this; }
    build() { return new FakeConnection(this.url); }
  }
  return { HubConnectionBuilder, LogLevel: { Warning: 3 } };
});

let mockTerm;
jest.mock("@xterm/xterm", () => ({
  Terminal: class {
    constructor() {
      this.cols = 100;
      this.rows = 30;
      this.write = jest.fn();
      this.focus = jest.fn();
      this.dispose = jest.fn();
      this.loadAddon = jest.fn();
      this.open = jest.fn();
      this.onData = (cb) => { this.type = cb; return { dispose: jest.fn() }; };
      this.onResize = (cb) => { this.resized = cb; return { dispose: jest.fn() }; };
      mockTerm = this;
    }
  },
}));
jest.mock("@xterm/addon-fit", () => ({ FitAddon: class { fit() {} } }));

const robot = { id: "7f2d12c8-c290-4681-aa4b-76521c57fd01", name: "blossom-07" };
const conn = () => mockConnections[mockConnections.length - 1];
const sent = (c, method) => c.send.mock.calls.filter(([m]) => m === method).map(([, ...args]) => args);

async function openTerminal(props = {}) {
  const view = render(<RobotTerminal robot={robot} onClose={jest.fn()} {...props} />);
  await waitFor(() => expect(screen.getByText(/^Connected/)).toBeInTheDocument());
  return view;
}

beforeEach(() => {
  mockConnections.length = 0;
  mockOnConnection = null;
  global.ResizeObserver = class { observe() {} disconnect() {} };
  useApiClient.mockReturnValue({ shellHubUrl: () => "https://bloom.example.edu/hubs/shell" });
});

describe("RobotTerminal", () => {
  it("connects to the hub and opens a session sized to the terminal", async () => {
    await openTerminal();

    expect(conn().url).toBe("https://bloom.example.edu/hubs/shell");
    expect(conn().invoke).toHaveBeenCalledWith("OpenSession", robot.id, 100, 30, false);
    expect(mockTerm.focus).toHaveBeenCalled();
  });

  it("writes server output to the terminal as raw bytes", async () => {
    await openTerminal();
    // A UTF-8 box-drawing character, an escape sequence, and a byte that isn't valid UTF-8 at all.
    const bytes = Uint8Array.from([0xe2, 0x94, 0x80, 0x1b, 0x5b, 0x33, 0x31, 0x6d, 0xff]);

    act(() => conn().emit("Output", bytesToBase64(bytes)));

    expect(Array.from(mockTerm.write.mock.calls[0][0])).toEqual(Array.from(bytes));
  });

  it("sends keystrokes as base64, and splits a big paste into small pieces", async () => {
    await openTerminal();

    act(() => mockTerm.type("é"));
    const [[first]] = sent(conn(), "Input");
    expect(Array.from(base64ToBytes(first))).toEqual([0xc3, 0xa9]);

    conn().send.mockClear();
    act(() => mockTerm.type("x".repeat(20000)));
    const pieces = sent(conn(), "Input").map(([b64]) => base64ToBytes(b64).length);
    expect(pieces).toEqual([8192, 8192, 3616]);  // never one message over the backend's receive limit
  });

  it("ignores typing until a session is open", async () => {
    render(<RobotTerminal robot={robot} onClose={jest.fn()} />);
    act(() => mockTerm.type("early"));
    expect(sent(conn(), "Input")).toEqual([]);
  });

  it("forwards terminal resizes", async () => {
    await openTerminal();
    act(() => mockTerm.resized({ cols: 132, rows: 43 }));
    expect(sent(conn(), "Resize")).toEqual([[132, 43]]);
  });

  it("explains why a session ended, stops the connection, and can reconnect", async () => {
    const user = userEvent.setup();
    await openTerminal();
    const first = conn();

    act(() => first.emit("SessionClosed", "taken_over"));

    expect(await screen.findByText(/another admin took over/i)).toBeInTheDocument();
    expect(first.stop).toHaveBeenCalled();
    expect(screen.getByText("Not connected")).toBeInTheDocument();

    await user.click(screen.getByRole("button", { name: "Reconnect" }));
    await waitFor(() => expect(screen.getByText(/^Connected/)).toBeInTheDocument());
    expect(mockConnections).toHaveLength(2);
    expect(conn()).not.toBe(first);
  });

  it("shows who holds a busy robot (not as a failure) and takes over with force on a new connection", async () => {
    const user = userEvent.setup();
    mockOnConnection = (c, index) => {
      if (index === 0) c.invokeResult = { status: "busy", heldBy: "alice@example.edu", since: "2026-10-29T10:00:00Z" };
    };
    render(<RobotTerminal robot={robot} onClose={jest.fn()} />);

    expect(await screen.findByText(/alice@example.edu/)).toBeInTheDocument();
    expect(mockConnections[0].stop).toHaveBeenCalled();
    expect(screen.getByText("Not connected")).toBeInTheDocument();
    expect(screen.queryByText("Connection closed.")).not.toBeInTheDocument();  // stopping it isn't an error

    await user.click(screen.getByRole("button", { name: "Take over" }));

    await waitFor(() => expect(screen.getByText(/^Connected/)).toBeInTheDocument());
    expect(mockConnections).toHaveLength(2);
    expect(conn().invoke).toHaveBeenCalledWith("OpenSession", robot.id, 100, 30, true);
    expect(screen.queryByText(/alice@example.edu/)).not.toBeInTheDocument();
  });

  it("reports refusals from the server and hangs up", async () => {
    mockOnConnection = (c) => {
      c.invokeResult = { status: "offline", message: "Robot is offline." };
    };
    render(<RobotTerminal robot={robot} onClose={jest.fn()} />);

    expect(await screen.findByText("Robot is offline.")).toBeInTheDocument();
    expect(conn().stop).toHaveBeenCalled();
    expect(screen.getByRole("button", { name: "Reconnect" })).toBeInTheDocument();
  });

  it("says so when the admin isn't allowed in", async () => {
    jest.spyOn(console, "error").mockImplementation(() => {});
    mockOnConnection = (c) => {
      c.start = jest.fn(async () => {
        throw new Error("Failed to complete negotiation with the server: Error: Forbidden: Status code '403'");
      });
    };
    render(<RobotTerminal robot={robot} onClose={jest.fn()} />);

    expect(await screen.findByText(/don't have permission/i)).toBeInTheDocument();
    expect(conn().invoke).not.toHaveBeenCalled();
  });

  it("handles an unreachable server", async () => {
    jest.spyOn(console, "error").mockImplementation(() => {});
    mockOnConnection = (c) => {
      c.start = jest.fn(async () => {
        throw new Error("Failed to fetch");
      });
    };
    render(<RobotTerminal robot={robot} onClose={jest.fn()} />);

    expect(await screen.findByText(/couldn't connect/i)).toBeInTheDocument();
  });

  it("ignores events from a connection it has replaced", async () => {
    const user = userEvent.setup();
    await openTerminal();
    const first = conn();
    act(() => first.emit("SessionClosed", "exited"));
    await user.click(await screen.findByRole("button", { name: "Reconnect" }));
    await waitFor(() => expect(screen.getByText(/^Connected/)).toBeInTheDocument());

    // A late message from the old connection must not leak into the new session.
    mockTerm.write.mockClear();
    act(() => first.emit("Output", bytesToBase64(new TextEncoder().encode("stale"))));
    act(() => first.emit("SessionClosed", "revoked"));

    expect(mockTerm.write).not.toHaveBeenCalled();
    expect(screen.getByText(/^Connected/)).toBeInTheDocument();
  });

  it("says goodbye when closed, so the audit log shows an admin close", async () => {
    const { unmount } = await openTerminal();
    const c = conn();

    unmount();

    await waitFor(() => expect(c.stop).toHaveBeenCalled());
    expect(sent(c, "CloseSession")).toHaveLength(1);
    expect(c.send.mock.invocationCallOrder[0]).toBeLessThan(c.stop.mock.invocationCallOrder[0]);
    expect(mockTerm.dispose).toHaveBeenCalled();
  });
});
