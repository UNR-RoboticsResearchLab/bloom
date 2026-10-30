import {
  buildDeployCommand,
  buildDeployCommandLines,
  defaultFlashOptions,
  isMsargModel,
  robotNumberFromName,
  shellPath,
  shellQuote,
  validateFlashOptions,
} from "./msargDeploy";

const complete = (overrides = {}) => ({
  ...defaultFlashOptions({ robotName: "m-sarg-04", fileName: "m-sarg-04.provision.json" }),
  device: "/dev/sdb",
  wifi: [{ kind: "psk", ssid: "lab wifi", psk: "it's secret", identity: "", password: "" }],
  envFile: "~/bloom/backend/.env",
  ...overrides,
});

describe("msargDeploy", () => {
  it("recognises M-SARG models and robot numbers in names", () => {
    expect(isMsargModel("M-SARG")).toBe(true);
    expect(isMsargModel("msarg")).toBe(true);
    expect(isMsargModel("Blossom")).toBe(false);
    expect(robotNumberFromName("m-sarg-04")).toBe("4");
    expect(robotNumberFromName("robot 100")).toBe("100");
    expect(robotNumberFromName("m-sarg-101")).toBe("");
    expect(robotNumberFromName("m-sarg-00")).toBe("");
    expect(robotNumberFromName("blossom")).toBe("");
  });

  it("quotes for the shell only when needed, keeping ~/ expandable", () => {
    expect(shellQuote("/dev/sdb")).toBe("/dev/sdb");
    expect(shellQuote("lab wifi")).toBe("'lab wifi'");
    expect(shellQuote("it's")).toBe("'it'\\''s'");
    expect(shellQuote("$(rm -rf /)")).toBe("'$(rm -rf /)'");
    expect(shellPath("~/.ssh/id_ed25519.pub")).toBe("~/.ssh/id_ed25519.pub");
    expect(shellPath("~/My Keys/k.pub")).toBe("~/'My Keys/k.pub'");
  });

  it("builds the deploy_sd.sh command with the bloom service and provisioning file", () => {
    expect(buildDeployCommandLines(complete())).toEqual([
      "sudo deploy/deploy_sd.sh",
      "--robot-id 4",
      "--device /dev/sdb",
      "--wifi-ssid 'lab wifi' --wifi-psk 'it'\\''s secret'",
      "--ssh-pubkey ~/.ssh/id_ed25519.pub",
      "--env-file ~/bloom/backend/.env",
      "--enable-bloom-service",
      "--bloom-provision-file ~/Downloads/m-sarg-04.provision.json",
      '--repo-token "$(gh auth token)"',
    ]);
    expect(buildDeployCommand(complete())).toContain(" \\\n  --robot-id 4");
  });

  it("handles image output, every Wi-Fi kind, no gh token and a master image", () => {
    const lines = buildDeployCommandLines(
      complete({
        target: "image",
        imagePath: "./m-robot-04.img",
        wifi: [
          { kind: "eap", ssid: "eduroam", identity: "me@unr.edu", password: "pw", psk: "" },
          { kind: "open", ssid: "Guest", psk: "", identity: "", password: "" },
          { kind: "psk", ssid: "  ", psk: "", identity: "", password: "" }, // blank rows are ignored
        ],
        useGhToken: false,
        masterImage: "~/m-sarg-images/master.img",
      })
    );
    expect(lines).toContain("--image ./m-robot-04.img");
    expect(lines).toContain("--wifi-eap-ssid eduroam --wifi-eap-identity me@unr.edu --wifi-eap-password pw");
    expect(lines).toContain("--wifi-open-ssid Guest");
    expect(lines.filter((l) => l.startsWith("--wifi"))).toHaveLength(2);
    expect(lines).not.toContain('--repo-token "$(gh auth token)"');
    expect(lines).toContain('--base-image-url "file://$HOME/"m-sarg-images/master.img');
  });

  it("accepts a complete form", () => {
    expect(validateFlashOptions(complete())).toEqual([]);
  });

  it("rejects what deploy_sd.sh would reject", () => {
    const errorsFor = (o) => validateFlashOptions(complete(o)).join(" ");
    expect(errorsFor({ robotNumber: "0" })).toMatch(/1 to 100/);
    expect(errorsFor({ robotNumber: "4.5" })).toMatch(/1 to 100/);
    expect(errorsFor({ device: "sdb" })).toMatch(/\/dev\/sdb/);
    expect(errorsFor({ target: "image" })).toMatch(/image file/);
    expect(errorsFor({ wifi: [] })).toMatch(/at least one Wi-Fi/);
    expect(errorsFor({ wifi: [{ kind: "psk", ssid: "x", psk: 'a"b' }] })).toMatch(/can't contain/);
    expect(errorsFor({ wifi: [{ kind: "psk", ssid: "x", psk: "" }] })).toMatch(/enter its password/);
    expect(errorsFor({ wifi: [{ kind: "eap", ssid: "x", identity: "u", password: "" }] })).toMatch(/username and password/);
    expect(errorsFor({ sshPubkey: "" })).toMatch(/SSH public key/);
    expect(errorsFor({ envFile: "" })).toMatch(/\.env/);
    expect(errorsFor({ masterImage: "master.img" })).toMatch(/absolute path/);
  });
});
