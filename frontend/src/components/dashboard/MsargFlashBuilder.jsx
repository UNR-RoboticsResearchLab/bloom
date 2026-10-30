// MsargFlashBuilder.jsx
// Form that writes the m-sarg repo's deploy/deploy_sd.sh command for flashing an M-SARG SD card with
// this robot's provisioning file: Wi-Fi, SSH key, Azure .env, card device or image file. The browser
// can't write the card, so the admin copies the command and runs it from their m-sarg checkout.
// Nothing typed here is sent anywhere.

import React, { useId, useMemo, useState } from "react";
import {
  WIFI_KINDS,
  buildDeployCommand,
  defaultFlashOptions,
  validateFlashOptions,
} from "../../utils/msargDeploy";

const inputClass =
  "mt-1 block w-full rounded-md border border-gray-300 px-2 py-1.5 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200";
const labelClass = "block text-xs font-medium text-gray-700";
const smallButtonClass =
  "rounded border border-gray-300 px-2 py-1 text-xs font-semibold text-gray-700 hover:bg-white focus:outline-none focus:ring-2 focus:ring-indigo-500";

const emptyNetwork = () => ({ kind: "psk", ssid: "", psk: "", identity: "", password: "" });

export default function MsargFlashBuilder({ robot, fileName, onCopy, copied }) {
  const id = useId();
  const [opts, setOpts] = useState(() => defaultFlashOptions({ robotName: robot.name, fileName }));

  const errors = useMemo(() => validateFlashOptions(opts), [opts]);
  const command = errors.length === 0 ? buildDeployCommand(opts) : "";
  const hostNumber = String(Number(opts.robotNumber) || 0).padStart(2, "0");

  const set = (key) => (e) => setOpts((o) => ({ ...o, [key]: e.target.type === "checkbox" ? e.target.checked : e.target.value }));
  const setNetwork = (i, key) => (e) =>
    setOpts((o) => ({ ...o, wifi: o.wifi.map((w, j) => (j === i ? { ...w, [key]: e.target.value } : w)) }));
  const addNetwork = () => setOpts((o) => ({ ...o, wifi: [...o.wifi, emptyNetwork()] }));
  const removeNetwork = (i) => setOpts((o) => ({ ...o, wifi: o.wifi.filter((_, j) => j !== i) }));

  const field = (key, label, props = {}) => (
    <div>
      <label htmlFor={`${id}-${key}`} className={labelClass}>
        {label}
      </label>
      <input id={`${id}-${key}`} type="text" value={opts[key]} onChange={set(key)} autoComplete="off" className={inputClass} {...props} />
    </div>
  );

  return (
    <div className="space-y-4">
      <p className="text-xs text-gray-600">
        Run this from your <span className="font-mono">m-sarg</span> checkout on a Linux machine with an SD card reader.
        It builds the whole image offline (20+ minutes the first time) and bakes in this robot's ID, secret and backend
        URL, so the robot connects to bloom on first boot with no setup on the robot.
      </p>

      <div className="grid grid-cols-2 gap-3">
        {field("robotNumber", "M-SARG robot number (1-100)", { inputMode: "numeric" })}
        <p className="self-end pb-1.5 text-xs text-gray-500">
          Hostname <span className="font-mono">m-robot-{hostNumber}</span>, ROS_DOMAIN_ID {Number(opts.robotNumber) || "?"}.
          Use a different number for each card.
        </p>
      </div>

      <fieldset>
        <legend className={labelClass}>Write to</legend>
        <div className="mt-1 flex gap-4 text-sm">
          {[
            ["device", "SD card"],
            ["image", "Image file"],
          ].map(([value, label]) => (
            <label key={value} className="flex items-center gap-1.5">
              <input type="radio" name={`${id}-target`} value={value} checked={opts.target === value} onChange={set("target")} />
              {label}
            </label>
          ))}
        </div>
        {opts.target === "device" ? (
          <>
            {field("device", "Card device", { placeholder: "/dev/sdb" })}
            <p className="mt-1 text-xs text-gray-500">
              Find it with <span className="font-mono">lsblk</span> after inserting the card. Everything on that device is
              erased; the script asks you to type the path again before writing.
            </p>
          </>
        ) : (
          field("imagePath", "Image file", { placeholder: "./m-robot-04.img" })
        )}
      </fieldset>

      <fieldset className="space-y-2">
        <legend className={labelClass}>Wi-Fi networks</legend>
        {opts.wifi.map((w, i) => (
          <div key={i} className="space-y-2 rounded border border-gray-200 bg-white p-2">
            <div className="flex items-end gap-2">
              <div className="w-36 shrink-0">
                <label htmlFor={`${id}-wifi-${i}-kind`} className={labelClass}>
                  Network {i + 1} security
                </label>
                <select id={`${id}-wifi-${i}-kind`} value={w.kind} onChange={setNetwork(i, "kind")} className={inputClass}>
                  {Object.entries(WIFI_KINDS).map(([value, label]) => (
                    <option key={value} value={value}>
                      {label}
                    </option>
                  ))}
                </select>
              </div>
              <div className="min-w-0 flex-1">
                <label htmlFor={`${id}-wifi-${i}-ssid`} className={labelClass}>
                  Network {i + 1} name (SSID)
                </label>
                <input id={`${id}-wifi-${i}-ssid`} type="text" value={w.ssid} onChange={setNetwork(i, "ssid")} autoComplete="off" className={inputClass} />
              </div>
              {opts.wifi.length > 1 && (
                <button type="button" onClick={() => removeNetwork(i)} className={smallButtonClass} aria-label={`Remove network ${i + 1}`}>
                  Remove
                </button>
              )}
            </div>
            {w.kind === "psk" && (
              <div>
                <label htmlFor={`${id}-wifi-${i}-psk`} className={labelClass}>
                  Network {i + 1} password
                </label>
                <input id={`${id}-wifi-${i}-psk`} type="password" value={w.psk} onChange={setNetwork(i, "psk")} autoComplete="off" className={inputClass} />
              </div>
            )}
            {w.kind === "eap" && (
              <div className="grid grid-cols-2 gap-2">
                <div>
                  <label htmlFor={`${id}-wifi-${i}-identity`} className={labelClass}>
                    Network {i + 1} username
                  </label>
                  <input id={`${id}-wifi-${i}-identity`} type="text" value={w.identity} onChange={setNetwork(i, "identity")} autoComplete="off" className={inputClass} />
                </div>
                <div>
                  <label htmlFor={`${id}-wifi-${i}-password`} className={labelClass}>
                    Network {i + 1} password
                  </label>
                  <input id={`${id}-wifi-${i}-password`} type="password" value={w.password} onChange={setNetwork(i, "password")} autoComplete="off" className={inputClass} />
                </div>
              </div>
            )}
          </div>
        ))}
        <button type="button" onClick={addNetwork} className={smallButtonClass}>
          Add another network
        </button>
        <p className="text-xs text-gray-500">
          The robot roams between these in order. More can be added later from its touchscreen settings panel.
        </p>
      </fieldset>

      <div className="grid grid-cols-2 gap-3">
        {field("sshPubkey", "SSH public key (on this machine)")}
        {field("envFile", "Azure speech .env file", { placeholder: "~/development/bloom/backend/.env" })}
      </div>
      {field("provisionFile", "Provisioning file (downloaded above)")}

      <label className="flex items-start gap-2 text-sm text-gray-800">
        <input
          type="checkbox"
          checked={opts.useGhToken}
          onChange={set("useGhToken")}
          className="mt-0.5 h-4 w-4 rounded border-gray-300 text-indigo-600 focus:ring-indigo-500"
        />
        <span>
          Authenticate the m-sarg clone with <span className="font-mono">gh auth token</span> (the repo is private)
        </span>
      </label>

      {field("masterImage", "Master image (optional, for faster repeat flashes)", { placeholder: "~/m-sarg-images/master.img" })}

      <div>
        <p className={labelClass}>Command</p>
        {command ? (
          <div className="mt-1 flex gap-2">
            <pre className="min-w-0 flex-1 overflow-x-auto whitespace-pre rounded bg-white px-2 py-1 font-mono text-xs" aria-label="deploy_sd.sh command">
              {command}
            </pre>
            <button type="button" onClick={() => onCopy("flash", command)} className={`shrink-0 self-start ${smallButtonClass}`}>
              {copied === "flash" ? "Copied" : "Copy"}
            </button>
          </div>
        ) : (
          <ul className="mt-1 list-disc space-y-0.5 pl-5 text-xs text-amber-800" aria-label="Still needed">
            {errors.map((err) => (
              <li key={err}>{err}</li>
            ))}
          </ul>
        )}
        <p className="mt-2 text-xs text-gray-600">
          The Wi-Fi passwords end up in the command (and your shell history). The card itself holds the robot's secret, so
          treat it like a password, and delete the provisioning file once it's flashed. Flash each card with its own
          robot: add a new one here for every card.
        </p>
      </div>
    </div>
  );
}
