import React from "react";
import { render, screen, waitFor, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";

import UserManagementCard from "../UserManagementCard";
import { useApiClient } from "../../../context/ApiClientContext";

jest.mock("../../../context/ApiClientContext", () => ({ useApiClient: jest.fn() }));

const USERS = [
  { id: "me", fullName: "Ada Admin", email: "ada@x.edu", role: "Admin", createdDate: "2026-01-01T00:00:00Z" },
  { id: "u2", fullName: "Sam SLP", email: "sam@x.edu", role: "SLP", createdDate: "2026-02-01T00:00:00Z" },
  { id: "u3", fullName: "Pat Participant", email: "pat@x.edu", role: "Participant", createdDate: "2026-03-01T00:00:00Z" },
];

let api;
beforeEach(() => {
  api = {
    getAdminUsers: jest.fn().mockResolvedValue(USERS),
    getAssignableRoles: jest.fn().mockResolvedValue(["Admin", "SLP", "Teacher", "Student"]),
    getMe: jest.fn().mockResolvedValue({ id: "me" }),
    setUserRole: jest.fn().mockResolvedValue({}),
    deleteAdminUser: jest.fn().mockResolvedValue({}),
    signUp: jest.fn().mockResolvedValue({}),
  };
  useApiClient.mockReturnValue(api);
  jest.spyOn(console, "error").mockImplementation(() => {});
});

describe("UserManagementCard", () => {
  it("lists users; own row and non-assignable roles can't be edited", async () => {
    render(<UserManagementCard />);
    expect(await screen.findByText("Sam SLP")).toBeInTheDocument();
    expect(screen.getByRole("combobox", { name: "Role for Sam SLP" })).toBeInTheDocument();
    expect(screen.queryByRole("combobox", { name: "Role for Ada Admin" })).not.toBeInTheDocument();
    expect(screen.queryByRole("combobox", { name: "Role for Pat Participant" })).not.toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Delete Ada Admin" })).toBeDisabled();
  });

  it("changes a role only after confirmation", async () => {
    const user = userEvent.setup();
    const onNotice = jest.fn();
    render(<UserManagementCard onNotice={onNotice} />);
    await user.selectOptions(await screen.findByRole("combobox", { name: "Role for Sam SLP" }), "Teacher");
    expect(api.setUserRole).not.toHaveBeenCalled();
    await user.click(screen.getByRole("button", { name: "Change role" }));
    await waitFor(() => expect(api.setUserRole).toHaveBeenCalledWith("u2", "Teacher"));
    expect(onNotice).toHaveBeenCalledWith("Sam SLP is now Teacher.");
    expect(api.getAdminUsers).toHaveBeenCalledTimes(2);
  });

  it("shows the backend's message when a change is refused", async () => {
    const user = userEvent.setup();
    api.setUserRole.mockRejectedValue(new Error('HTTP 409: {"message":"That\'s the last administrator."}'));
    render(<UserManagementCard />);
    await user.selectOptions(await screen.findByRole("combobox", { name: "Role for Sam SLP" }), "Student");
    await user.click(screen.getByRole("button", { name: "Change role" }));
    expect(await screen.findByRole("alert")).toHaveTextContent("last administrator");
    expect(screen.getByRole("dialog")).toBeInTheDocument();
  });

  it("deletes after confirmation", async () => {
    const user = userEvent.setup();
    render(<UserManagementCard />);
    await user.click(await screen.findByRole("button", { name: "Delete Sam SLP" }));
    const dialog = screen.getByRole("dialog");
    await user.click(within(dialog).getByRole("button", { name: "Delete user" }));
    await waitFor(() => expect(api.deleteAdminUser).toHaveBeenCalledWith("u2"));
  });

  it("filters by search and creates a user with the chosen role", async () => {
    const user = userEvent.setup();
    render(<UserManagementCard />);
    await screen.findByText("Sam SLP");
    await user.type(screen.getByLabelText("Search users"), "pat@");
    expect(screen.queryByText("Sam SLP")).not.toBeInTheDocument();
    await user.clear(screen.getByLabelText("Search users"));

    await user.click(screen.getByRole("button", { name: "Add user" }));
    await user.type(screen.getByLabelText("Full name"), "New Person");
    await user.type(screen.getByLabelText("Email"), "new@x.edu");
    await user.type(screen.getByLabelText("Temporary password"), "Passw0rd!");
    await user.selectOptions(screen.getByLabelText("Role"), "Student");
    await user.click(screen.getByRole("button", { name: "Create user" }));
    await waitFor(() =>
      expect(api.signUp).toHaveBeenCalledWith({ fullName: "New Person", email: "new@x.edu", password: "Passw0rd!", role: "Student" })
    );
  });
});
