import {
  apiErrorMessage,
  apiErrorStatus,
  buildProvisionFile,
  provisionFileName,
} from "./provisioning";

describe("provisioning file", () => {
  it("has exactly the fields deploy/provision.sh reads", () => {
    const file = buildProvisionFile({
      backendUrl: "https://bloom.example.edu",
      robot: { id: "7f2d12c8-c290-4681-aa4b-76521c57fd01", name: "blossom-07", ignored: true },
      secret: "s3cret",
    });
    expect(file).toEqual({
      version: 1,
      backend_url: "https://bloom.example.edu",
      robot_id: "7f2d12c8-c290-4681-aa4b-76521c57fd01",
      name: "blossom-07",
      secret: "s3cret",
    });
  });

  it("names the file after the robot, safely", () => {
    expect(provisionFileName("blossom-07")).toBe("blossom-07.provision.json");
    expect(provisionFileName("Lab Robot #3")).toBe("lab-robot-3.provision.json");
    // No path separators survive, so the browser can't be talked into saving outside Downloads.
    expect(provisionFileName("../../etc/passwd")).toBe("..-..-etc-passwd.provision.json");
    expect(provisionFileName("")).toBe("robot.provision.json");
    expect(provisionFileName(undefined)).toBe("robot.provision.json");
  });
});

describe("API errors", () => {
  it("reads the backend's message", () => {
    const err = new Error('HTTP 502: {"message":"The robot couldn\'t apply it. Nothing changed."}');
    expect(apiErrorStatus(err)).toBe(502);
    expect(apiErrorMessage(err, "fallback")).toBe("The robot couldn't apply it. Nothing changed.");
  });

  it("falls back for anything else", () => {
    expect(apiErrorMessage(new Error("HTTP 500: <html>"), "fallback")).toBe("fallback");
    expect(apiErrorMessage(new Error("network down"), "fallback")).toBe("fallback");
    expect(apiErrorStatus(new Error("network down"))).toBeNull();
    expect(apiErrorMessage(undefined, "fallback")).toBe("fallback");
  });
});
