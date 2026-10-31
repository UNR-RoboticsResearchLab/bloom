// Helpers for the robot provisioning file: what the dashboard hands an admin so a robot can be given
// its credentials offline (USB stick, scp). deploy/provision.sh on the robot consumes it. It contains
// the robot's secret in plain text.

export const PROVISION_FILE_VERSION = 1;

export function buildProvisionFile({ backendUrl, robot, secret }) {
  return {
    version: PROVISION_FILE_VERSION,
    backend_url: backendUrl,
    robot_id: robot.id,
    name: robot.name,
    secret,
  };
}

export function provisionFileName(robotName) {
  const slug = String(robotName || "")
    .toLowerCase()
    .replace(/[^a-z0-9._-]+/g, "-")
    .replace(/^-+|-+$/g, "");
  return `${slug || "robot"}.provision.json`;
}

export function downloadProvisionFile(file) {
  const blob = new Blob([JSON.stringify(file, null, 2) + "\n"], { type: "application/json" });
  const url = URL.createObjectURL(blob);
  const link = document.createElement("a");
  link.href = url;
  link.download = provisionFileName(file.name);
  document.body.appendChild(link);
  link.click();
  link.remove();
  URL.revokeObjectURL(url);
}

// ApiClient.request throws `HTTP <status>: <body>`; the backend's errors are {"message": "..."}.
export function apiErrorStatus(err) {
  const m = /^HTTP (\d{3})/.exec(err?.message || "");
  return m ? Number(m[1]) : null;
}

export function apiErrorMessage(err, fallback) {
  const body = /^HTTP \d{3}: ([\s\S]*)$/.exec(err?.message || "")?.[1];
  if (body) {
    try {
      const parsed = JSON.parse(body);
      const text = parsed.message || parsed.Message;
      if (typeof text === "string" && text) return text;
    } catch {
      // not JSON
    }
  }
  return fallback;
}
