import React from "react";
import { render, screen, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";

import RobotFleetCard from "../RobotFleetCard";

const robot = { id: "r1", name: "m-sarg-04", model: "M-SARG", online: true, agentVersion: "1.2.0-1" };
const releases = { prod: ["1.2.0-1", "1.1.0-1"], canary: ["1.3.0~rc1-1"], dev: [] };

describe("RobotFleetCard version picker", () => {
  it("lists the channel's releases newest first and marks the running one", async () => {
    const user = userEvent.setup();
    render(<RobotFleetCard robot={robot} releases={releases} onUpdate={jest.fn()} />);

    const select = screen.getByRole("combobox", { name: "Target version" });
    expect(within(select).getAllByRole("option").map((o) => o.textContent)).toEqual(["1.2.0-1 (current)", "1.1.0-1"]);
    expect(screen.getByRole("button", { name: "Already on this version" })).toBeDisabled();

    await user.click(screen.getByRole("radio", { name: "Canary" }));
    expect(select).toHaveValue("1.3.0~rc1-1");
    expect(screen.getByRole("button", { name: "Update to 1.3.0~rc1-1" })).toBeEnabled();

    await user.click(screen.getByRole("radio", { name: "Dev" }));
    expect(select).toBeDisabled();
    expect(within(select).getByRole("option")).toHaveTextContent("No dev builds");
  });
});
