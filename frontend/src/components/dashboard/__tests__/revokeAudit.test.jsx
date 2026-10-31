import React from "react";
import { render, screen, waitFor } from "@testing-library/react";
import userEvent from "@testing-library/user-event";

import RevokeRobotDialog from "../RevokeRobotDialog";
import ShellAuditDialog from "../ShellAuditDialog";
import { useApiClient } from "../../../context/ApiClientContext";

jest.mock("../../../context/ApiClientContext", () => ({ useApiClient: jest.fn() }));

const robot = { id: "7f2d12c8-c290-4681-aa4b-76521c57fd01", name: "blossom-07", online: true, provisioned: true };

let api;
beforeEach(() => {
  api = {
    revokeRobot: jest.fn().mockResolvedValue({}),
    getShellSessions: jest.fn().mockResolvedValue([]),
  };
  useApiClient.mockReturnValue(api);
});

describe("RevokeRobotDialog", () => {
  it("starts on Cancel and revokes on confirm", async () => {
    const user = userEvent.setup();
    const onRevoked = jest.fn();
    const onClose = jest.fn();
    render(<RevokeRobotDialog robot={robot} onClose={onClose} onRevoked={onRevoked} />);
    expect(screen.getByRole("button", { name: "Cancel" })).toHaveFocus();
    await user.click(screen.getByRole("button", { name: "Revoke robot" }));
    await waitFor(() => expect(onClose).toHaveBeenCalled());
    expect(api.revokeRobot).toHaveBeenCalledWith(robot.id);
    expect(onRevoked).toHaveBeenCalledWith(robot);
  });

  it("shows an error and stays open when revoking fails", async () => {
    const user = userEvent.setup();
    api.revokeRobot.mockRejectedValue(new Error("boom"));
    jest.spyOn(console, "error").mockImplementation(() => {});
    const onClose = jest.fn();
    render(<RevokeRobotDialog robot={robot} onClose={onClose} />);
    await user.click(screen.getByRole("button", { name: "Revoke robot" }));
    expect(await screen.findByRole("alert")).toHaveTextContent(/couldn't revoke/i);
    expect(onClose).not.toHaveBeenCalled();
  });
});

describe("ShellAuditDialog", () => {
  it("lists sessions with a readable end reason", async () => {
    api.getShellSessions.mockResolvedValue([
      { id: "1", adminEmail: "a@x.edu", startedAt: "2026-10-30T10:00:00Z", endedAt: "2026-10-30T10:02:05Z", endReason: "taken_over" },
      { id: "2", adminUserId: "u2", startedAt: "2026-10-30T11:00:00Z", endedAt: null, endReason: null },
    ]);
    render(<ShellAuditDialog robot={robot} onClose={jest.fn()} />);
    expect(await screen.findByText("a@x.edu")).toBeInTheDocument();
    expect(screen.getByText("2m 5s")).toBeInTheDocument();
    expect(screen.getByText("Taken over by another admin")).toBeInTheDocument();
    expect(screen.getByText("Open now")).toBeInTheDocument();
    expect(api.getShellSessions).toHaveBeenCalledWith(robot.id);
  });

  it("handles empty and failed loads", async () => {
    const { unmount } = render(<ShellAuditDialog robot={robot} onClose={jest.fn()} />);
    expect(await screen.findByText(/no shell sessions yet/i)).toBeInTheDocument();
    unmount();
    jest.spyOn(console, "error").mockImplementation(() => {});
    api.getShellSessions.mockRejectedValue(new Error("x"));
    render(<ShellAuditDialog robot={robot} onClose={jest.fn()} />);
    expect(await screen.findByRole("alert")).toHaveTextContent(/couldn't load/i);
  });
});
