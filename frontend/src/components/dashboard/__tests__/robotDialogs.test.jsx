import React from "react";
import { render, screen, waitFor, within } from "@testing-library/react";
import userEvent from "@testing-library/user-event";

import AddRobotDialog from "../AddRobotDialog";
import DeleteRobotDialog from "../DeleteRobotDialog";
import RobotCredentialsDialog from "../RobotCredentialsDialog";
import { useApiClient } from "../../../context/ApiClientContext";

jest.mock("../../../context/ApiClientContext", () => ({ useApiClient: jest.fn() }));

const SECRET = "Xq3_SECRETvalue-0123456789abcdefghij";
const robot = (overrides = {}) => ({
  id: "7f2d12c8-c290-4681-aa4b-76521c57fd01",
  name: "blossom-07",
  online: true,
  provisioned: true,
  ...overrides,
});

let api;
beforeEach(() => {
  api = {
    publicBaseUrl: () => "https://bloom.example.edu",
    registerRobot: jest.fn(),
    deleteRobot: jest.fn().mockResolvedValue({}),
    pushRobotCredentials: jest.fn().mockResolvedValue({}),
    rotateRobotCredentials: jest.fn().mockResolvedValue({ secret: SECRET }),
  };
  useApiClient.mockReturnValue(api);
  global.URL.createObjectURL = jest.fn(() => "blob:fake");
  global.URL.revokeObjectURL = jest.fn();
});

describe("DeleteRobotDialog", () => {
  it("is a labelled modal that starts on Cancel, not on the destructive button", () => {
    render(<DeleteRobotDialog robot={robot()} onClose={jest.fn()} />);
    const dialog = screen.getByRole("dialog", { name: "Delete blossom-07?" });
    expect(dialog).toHaveAttribute("aria-modal", "true");
    expect(screen.getByRole("button", { name: "Cancel" })).toHaveFocus();
  });

  it("only deletes once the robot's name is typed exactly", async () => {
    const user = userEvent.setup();
    const onDeleted = jest.fn();
    const onClose = jest.fn();
    render(<DeleteRobotDialog robot={robot()} onClose={onClose} onDeleted={onDeleted} />);
    const confirm = screen.getByRole("button", { name: "Delete robot" });
    const input = screen.getByLabelText(/type blossom-07 to confirm/i);

    expect(confirm).toBeDisabled();
    await user.type(input, "blossom-0");
    expect(confirm).toBeDisabled();
    await user.type(input, "8");
    expect(confirm).toBeDisabled();
    await user.clear(input);
    await user.type(input, "blossom-07");
    expect(confirm).toBeEnabled();

    await user.click(confirm);
    expect(api.deleteRobot).toHaveBeenCalledWith("7f2d12c8-c290-4681-aa4b-76521c57fd01");
    await waitFor(() => expect(onDeleted).toHaveBeenCalledWith(expect.objectContaining({ name: "blossom-07" })));
    expect(onClose).toHaveBeenCalled();
  });

  it("can't be submitted with Enter while the name doesn't match", async () => {
    const user = userEvent.setup();
    render(<DeleteRobotDialog robot={robot()} onClose={jest.fn()} />);
    await user.type(screen.getByLabelText(/type blossom-07/i), "nope{Enter}");
    expect(api.deleteRobot).not.toHaveBeenCalled();
  });

  it("stays open and says why when the delete fails", async () => {
    const user = userEvent.setup();
    api.deleteRobot.mockRejectedValue(new Error('HTTP 403: {"message":"nope"}'));
    const onClose = jest.fn();
    render(<DeleteRobotDialog robot={robot()} onClose={onClose} />);
    await user.type(screen.getByLabelText(/type blossom-07/i), "blossom-07");
    await user.click(screen.getByRole("button", { name: "Delete robot" }));

    expect(await screen.findByRole("alert")).toHaveTextContent("don't have permission");
    expect(onClose).not.toHaveBeenCalled();
    expect(screen.getByRole("button", { name: "Delete robot" })).toBeEnabled();
  });

  it("warns about the live connection only for an online robot", () => {
    const { unmount } = render(<DeleteRobotDialog robot={robot()} onClose={jest.fn()} />);
    expect(screen.getByText(/connection and any open shell are closed/i)).toBeInTheDocument();
    unmount();
    render(<DeleteRobotDialog robot={robot({ online: false })} onClose={jest.fn()} />);
    expect(screen.queryByText(/connection and any open shell are closed/i)).not.toBeInTheDocument();
  });

  it("closes on Escape and returns focus to what opened it", async () => {
    const user = userEvent.setup();
    const opener = document.createElement("button");
    document.body.appendChild(opener);
    opener.focus();
    const onClose = jest.fn();
    const { unmount } = render(<DeleteRobotDialog robot={robot()} onClose={onClose} />);

    await user.keyboard("{Escape}");
    expect(onClose).toHaveBeenCalled();
    unmount();
    expect(opener).toHaveFocus();
    opener.remove();
  });

  it("keeps Tab inside the dialog", async () => {
    const user = userEvent.setup();
    render(<DeleteRobotDialog robot={robot()} onClose={jest.fn()} />);
    const dialog = screen.getByRole("dialog");
    for (let i = 0; i < 6; i++) {
      await user.tab();
      expect(dialog).toContainElement(document.activeElement);
    }
    for (let i = 0; i < 6; i++) {
      await user.tab({ shift: true });
      expect(dialog).toContainElement(document.activeElement);
    }
  });
});

describe("RobotCredentialsDialog", () => {
  it("defaults to pushing for an online robot, and never shows the secret", async () => {
    const user = userEvent.setup();
    const onChanged = jest.fn();
    render(<RobotCredentialsDialog robot={robot()} onClose={jest.fn()} onChanged={onChanged} />);
    expect(screen.getByRole("radio", { name: /push to the robot now/i })).toBeChecked();

    await user.click(screen.getByRole("button", { name: "Push credentials" }));

    expect(api.pushRobotCredentials).toHaveBeenCalledWith("7f2d12c8-c290-4681-aa4b-76521c57fd01");
    expect(api.rotateRobotCredentials).not.toHaveBeenCalled();
    expect(await screen.findByRole("heading", { name: "Credentials delivered" })).toBeInTheDocument();
    expect(screen.queryByText(SECRET)).not.toBeInTheDocument();
  });

  it("offers only the file for an offline robot", () => {
    render(<RobotCredentialsDialog robot={robot({ online: false })} onClose={jest.fn()} />);
    expect(screen.getByRole("radio", { name: /push to the robot now/i })).toBeDisabled();
    expect(screen.getByRole("radio", { name: /download a provisioning file/i })).toBeChecked();
    expect(screen.getByText(/the robot is offline/i)).toBeInTheDocument();
  });

  it("reports why a push failed and points at the file", async () => {
    const user = userEvent.setup();
    api.pushRobotCredentials.mockRejectedValue(
      new Error('HTTP 502: {"message":"The robot couldn\'t apply it: sudo: a password is required. Nothing changed."}')
    );
    render(<RobotCredentialsDialog robot={robot()} onClose={jest.fn()} />);
    await user.click(screen.getByRole("button", { name: "Push credentials" }));

    const alert = await screen.findByRole("alert");
    expect(alert).toHaveTextContent("sudo: a password is required");
    expect(alert).toHaveTextContent("download a provisioning file instead");
    expect(screen.getByRole("button", { name: "Push credentials" })).toBeEnabled();  // can retry
  });

  it("file path: shows the secret once and can't be closed until it's acknowledged", async () => {
    const user = userEvent.setup();
    const onClose = jest.fn();
    render(<RobotCredentialsDialog robot={robot({ online: false })} onClose={onClose} />);
    await user.click(screen.getByRole("button", { name: "Issue credentials" }));

    expect(await screen.findByDisplayValue(SECRET)).toBeInTheDocument();
    expect(screen.queryByRole("button", { name: "Close" })).not.toBeInTheDocument();  // no ×
    const done = screen.getByRole("button", { name: "Done" });
    expect(done).toBeDisabled();

    await user.keyboard("{Escape}");
    expect(onClose).not.toHaveBeenCalled();  // Escape can't throw the secret away
    await user.click(screen.getByRole("dialog").parentElement.firstChild);  // backdrop
    expect(onClose).not.toHaveBeenCalled();

    await user.click(screen.getByRole("checkbox", { name: /saved the secret/i }));
    expect(done).toBeEnabled();
    await user.click(done);
    expect(onClose).toHaveBeenCalled();
  });

  it("downloads a provisioning file that deploy/provision.sh can read", async () => {
    const user = userEvent.setup();
    let blob;
    global.URL.createObjectURL = jest.fn((b) => ((blob = b), "blob:fake"));
    render(<RobotCredentialsDialog robot={robot({ online: false })} onClose={jest.fn()} />);
    await user.click(screen.getByRole("button", { name: "Issue credentials" }));
    await user.click(await screen.findByRole("button", { name: "Download provisioning file" }));

    const file = JSON.parse(await new Response(blob).text());
    expect(file).toEqual({
      version: 1,
      backend_url: "https://bloom.example.edu",
      robot_id: "7f2d12c8-c290-4681-aa4b-76521c57fd01",
      name: "blossom-07",
      secret: SECRET,
    });
  });

  it("warns when the page isn't https, since the agent only connects over TLS", async () => {
    const user = userEvent.setup();
    api.publicBaseUrl = () => "http://localhost:3000";
    render(<RobotCredentialsDialog robot={robot({ online: false })} onClose={jest.fn()} />);
    await user.click(screen.getByRole("button", { name: "Issue credentials" }));
    expect(await screen.findByRole("note")).toHaveTextContent("isn't served over https");
  });
});

describe("AddRobotDialog", () => {
  it("requires a name", async () => {
    const user = userEvent.setup();
    render(<AddRobotDialog onClose={jest.fn()} />);
    await user.click(screen.getByRole("button", { name: "Add robot" }));
    expect(await screen.findByRole("alert")).toHaveTextContent("Enter a name");
    expect(api.registerRobot).not.toHaveBeenCalled();
    expect(screen.getByLabelText(/^name/i)).toHaveFocus();
  });

  it("registers the robot, then holds the secret behind an acknowledgement", async () => {
    const user = userEvent.setup();
    api.registerRobot.mockResolvedValue({ robot: { id: "abc", name: "blossom-08" }, secret: SECRET });
    const onClose = jest.fn();
    const onCreated = jest.fn();
    render(<AddRobotDialog onClose={onClose} onCreated={onCreated} />);

    await user.type(screen.getByLabelText(/^name/i), "  blossom-08 ");
    await user.click(screen.getByRole("button", { name: "Add robot" }));

    expect(api.registerRobot).toHaveBeenCalledWith({ name: "blossom-08", model: "Blossom" });
    expect(await screen.findByDisplayValue(SECRET)).toBeInTheDocument();
    expect(screen.getByRole("button", { name: "Done" })).toBeDisabled();

    await user.click(screen.getByRole("checkbox", { name: /saved the secret/i }));
    await user.click(screen.getByRole("button", { name: "Done" }));
    expect(onCreated).toHaveBeenCalledWith({ id: "abc", name: "blossom-08" });
    expect(onClose).toHaveBeenCalled();
  });

  it("builds the SD-card flash command for an M-SARG robot", async () => {
    const user = userEvent.setup();
    api.registerRobot.mockResolvedValue({ robot: { id: "abc", name: "m-sarg-04" }, secret: SECRET });
    render(<AddRobotDialog onClose={jest.fn()} />);

    await user.type(screen.getByLabelText(/^name/i), "m-sarg-04");
    await user.clear(screen.getByLabelText(/^model/i));
    await user.type(screen.getByLabelText(/^model/i), "M-SARG");
    await user.click(screen.getByRole("button", { name: "Add robot" }));
    expect(api.registerRobot).toHaveBeenCalledWith({ name: "m-sarg-04", model: "M-SARG" });

    // The flash section opens for M-SARG and starts with the robot number from the name.
    expect(await screen.findByLabelText(/m-sarg robot number/i)).toHaveValue("4");
    expect(screen.getByRole("list", { name: "Still needed" })).toBeInTheDocument();

    await user.type(screen.getByLabelText(/card device/i), "/dev/sdb");
    await user.type(screen.getByLabelText(/network 1 name/i), "lab");
    await user.type(screen.getByLabelText(/network 1 password/i), "pw12345678");
    await user.type(screen.getByLabelText(/azure speech \.env/i), "~/bloom.env");

    const command = screen.getByLabelText("deploy_sd.sh command");
    expect(command).toHaveTextContent("--robot-id 4");
    expect(command).toHaveTextContent("--device /dev/sdb");
    expect(command).toHaveTextContent("--wifi-ssid lab --wifi-psk pw12345678");
    expect(command).toHaveTextContent("--bloom-provision-file ~/Downloads/m-sarg-04.provision.json");
  });

  it("keeps the flash section closed for other models", async () => {
    const user = userEvent.setup();
    api.registerRobot.mockResolvedValue({ robot: { id: "abc", name: "blossom-08" }, secret: SECRET });
    render(<AddRobotDialog onClose={jest.fn()} />);
    await user.type(screen.getByLabelText(/^name/i), "blossom-08");
    await user.click(screen.getByRole("button", { name: "Add robot" }));
    await screen.findByDisplayValue(SECRET);
    expect(screen.getByText("Flash an M-SARG SD card").closest("details")).not.toHaveAttribute("open");
  });

  it("explains a rejected registration without losing the form", async () => {
    const user = userEvent.setup();
    api.registerRobot.mockRejectedValue(new Error("HTTP 400: {}"));
    render(<AddRobotDialog onClose={jest.fn()} />);
    await user.type(screen.getByLabelText(/^name/i), "dup");
    await user.click(screen.getByRole("button", { name: "Add robot" }));

    expect(await screen.findByRole("alert")).toHaveTextContent("isn't already in use");
    expect(screen.getByLabelText(/^name/i)).toHaveValue("dup");
  });
});
