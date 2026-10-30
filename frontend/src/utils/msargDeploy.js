// Builds the m-sarg repo's deploy/deploy_sd.sh command that flashes an M-SARG SD card with a bloom
// robot's credentials baked in. The browser can't write a card itself: the admin runs the command
// (as root) on a Linux machine with a card reader and the m-sarg checkout. Nothing here is sent to
// the backend; Wi-Fi passwords typed into the form only end up in the command text.
//
// The checks mirror deploy/lib/args.sh's validate_args, so a bad value fails here instead of after
// the script has started.

export const MIN_ROBOT_NUMBER = 1;
export const MAX_ROBOT_NUMBER = 100;

export const WIFI_KINDS = {
  psk: "WPA2 password",
  eap: "WPA2-Enterprise",
  open: "Open",
};

export function isMsargModel(model) {
  return /m-?sarg/i.test(String(model || ""));
}

// "m-sarg-04" -> 4, so a robot named the way flash_batch.sh names files gets its number filled in.
export function robotNumberFromName(name) {
  const m = /(\d+)\s*$/.exec(String(name || ""));
  if (!m) return "";
  const n = Number(m[1]);
  return n >= MIN_ROBOT_NUMBER && n <= MAX_ROBOT_NUMBER ? String(n) : "";
}

const SAFE = /^[A-Za-z0-9_@%+=:,./-]+$/;

export function shellQuote(value) {
  const s = String(value);
  return SAFE.test(s) ? s : `'${s.replace(/'/g, `'\\''`)}'`;
}

// Like shellQuote, but leaves a leading ~/ unquoted so the admin's shell still expands it (before
// sudo, so it is their home, not root's).
export function shellPath(path) {
  const p = String(path);
  if (p === "~") return p;
  if (p.startsWith("~/")) return p.length > 2 ? `~/${shellQuote(p.slice(2))}` : "~/";
  return shellQuote(p);
}

// wpa_supplicant.conf gets these values unescaped, so deploy_sd.sh refuses " and \ in them.
const UNSAFE_WIFI = /["\\]/;

export function defaultFlashOptions({ robotName, fileName }) {
  return {
    robotNumber: robotNumberFromName(robotName),
    target: "device", // or "image"
    device: "",
    imagePath: "",
    wifi: [{ kind: "psk", ssid: "", psk: "", identity: "", password: "" }],
    sshPubkey: "~/.ssh/id_ed25519.pub",
    envFile: "",
    provisionFile: `~/Downloads/${fileName}`,
    useGhToken: true,
    masterImage: "",
  };
}

export function validateFlashOptions(o) {
  const errors = [];
  const n = Number(o.robotNumber);
  if (!/^\d+$/.test(String(o.robotNumber).trim()) || n < MIN_ROBOT_NUMBER || n > MAX_ROBOT_NUMBER) {
    errors.push(`Robot number must be a whole number from ${MIN_ROBOT_NUMBER} to ${MAX_ROBOT_NUMBER}.`);
  }
  if (o.target === "device") {
    if (!/^\/dev\/\S+$/.test(o.device.trim())) errors.push("Enter the SD card's device, e.g. /dev/sdb.");
  } else if (!o.imagePath.trim()) {
    errors.push("Enter a path for the image file, e.g. ./m-robot-04.img.");
  }

  const networks = o.wifi.filter((w) => w.ssid.trim());
  if (networks.length === 0) errors.push("Add at least one Wi-Fi network.");
  networks.forEach((w) => {
    const values = [w.ssid, w.kind === "psk" && w.psk, w.kind === "eap" && w.identity, w.kind === "eap" && w.password];
    if (values.some((v) => v && UNSAFE_WIFI.test(v))) {
      errors.push(`Wi-Fi "${w.ssid}": names and passwords can't contain " or \\.`);
    }
    if (w.kind === "psk" && !w.psk) errors.push(`Wi-Fi "${w.ssid}": enter its password.`);
    if (w.kind === "eap" && (!w.identity || !w.password)) {
      errors.push(`Wi-Fi "${w.ssid}": enter a username and password.`);
    }
  });

  if (!o.sshPubkey.trim()) errors.push("Enter the SSH public key path. The card only allows key logins.");
  if (!o.envFile.trim()) errors.push("Enter the path of the .env file with the Azure speech keys.");
  if (!o.provisionFile.trim()) errors.push("Enter where the provisioning file is.");
  const master = o.masterImage.trim();
  if (master && !/^(\/|~\/|[a-z]+:\/\/)/.test(master)) {
    errors.push("The master image needs an absolute path (or a URL).");
  }
  return errors;
}

// Returns the command as lines; join with " \\\n" for a copyable multi-line command.
export function buildDeployCommandLines(o) {
  const lines = ["sudo deploy/deploy_sd.sh", `--robot-id ${Number(o.robotNumber)}`];
  lines.push(o.target === "device" ? `--device ${shellPath(o.device.trim())}` : `--image ${shellPath(o.imagePath.trim())}`);

  o.wifi
    .filter((w) => w.ssid.trim())
    .forEach((w) => {
      const ssid = shellQuote(w.ssid);
      if (w.kind === "psk") lines.push(`--wifi-ssid ${ssid} --wifi-psk ${shellQuote(w.psk)}`);
      else if (w.kind === "eap") {
        lines.push(
          `--wifi-eap-ssid ${ssid} --wifi-eap-identity ${shellQuote(w.identity)} --wifi-eap-password ${shellQuote(w.password)}`
        );
      } else lines.push(`--wifi-open-ssid ${ssid}`);
    });

  lines.push(`--ssh-pubkey ${shellPath(o.sshPubkey.trim())}`);
  lines.push(`--env-file ${shellPath(o.envFile.trim())}`);
  lines.push("--enable-bloom-service");
  lines.push(`--bloom-provision-file ${shellPath(o.provisionFile.trim())}`);
  if (o.useGhToken) lines.push('--repo-token "$(gh auth token)"');
  const master = o.masterImage.trim();
  if (master) {
    const url = master.startsWith("~/")
      ? `"file://$HOME/"${shellQuote(master.slice(2))}`
      : shellQuote(/^[a-z]+:\/\//.test(master) ? master : `file://${master}`);
    lines.push(`--base-image-url ${url}`);
  }
  return lines;
}

export function buildDeployCommand(o) {
  return buildDeployCommandLines(o).join(" \\\n  ");
}
