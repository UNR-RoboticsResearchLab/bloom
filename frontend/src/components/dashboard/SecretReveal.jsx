// SecretReveal.jsx
// Shows a robot's secret -- which the backend hands out exactly once -- with the ways to get it onto
// the robot: copy it, or download a provisioning file for deploy/provision.sh. The caller gates
// closing on the "I've saved it" checkbox via `saved` / `onSavedChange`.

import React, { useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import { buildProvisionFile, downloadProvisionFile, provisionFileName } from "../../utils/provisioning";

export default function SecretReveal({ robot, secret, saved, onSavedChange }) {
  const api = useApiClient();
  const secretId = useId();
  const [copied, setCopied] = useState("");
  const [error, setError] = useState("");

  const backendUrl = api.publicBaseUrl();
  const fileName = provisionFileName(robot.name);
  const insecure = !backendUrl.startsWith("https://");

  async function copy(label, text) {
    try {
      await navigator.clipboard.writeText(text);
      setError("");
      setCopied(label);
      setTimeout(() => setCopied(""), 2000);
    } catch {
      // Clipboard needs a secure context; the value is also selectable text.
      setError("Couldn't copy automatically. Select the text and copy it manually.");
    }
  }

  function download() {
    downloadProvisionFile(buildProvisionFile({ backendUrl, robot, secret }));
  }

  const buttonClass =
    "shrink-0 rounded-md border border-gray-300 px-3 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500";

  return (
    <div className="space-y-4">
      <dl className="grid grid-cols-[auto,1fr] gap-x-4 gap-y-2 text-sm">
        <dt className="font-medium text-gray-500">Name</dt>
        <dd className="text-gray-900">{robot.name}</dd>
        <dt className="font-medium text-gray-500">Robot ID</dt>
        <dd className="break-all font-mono text-gray-900">{robot.id}</dd>
      </dl>

      <div>
        <label htmlFor={secretId} className="block text-sm font-medium text-gray-900">
          Secret
        </label>
        <div className="mt-1 flex gap-2">
          <input
            id={secretId}
            type="text"
            readOnly
            value={secret}
            onFocus={(e) => e.target.select()}
            className="min-w-0 flex-1 rounded-md border border-gray-300 bg-gray-50 px-3 py-2 font-mono text-sm"
          />
          <button type="button" onClick={() => copy("secret", secret)} className={buttonClass}>
            {copied === "secret" ? "Copied" : "Copy"}
          </button>
        </div>
      </div>

      <div className="rounded-md bg-gray-50 px-3 py-3 text-sm">
        <p className="font-medium text-gray-900">Get it onto the robot</p>
        <button
          type="button"
          onClick={download}
          className="mt-2 rounded-md bg-indigo-600 px-3 py-2 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1"
        >
          Download provisioning file
        </button>
        <p className="mt-2 text-xs text-gray-600">
          <span className="font-mono">{fileName}</span> holds the secret in plain text. Move it to the robot
          (USB stick or scp), run the command below, and delete it.
        </p>
        {insecure && (
          <p role="note" className="mt-2 text-xs text-amber-800">
            This page isn't served over https, so the file's backend_url ({backendUrl}) won't be accepted. The
            agent only connects over TLS. Edit it to the backend's https address.
          </p>
        )}
        <details className="mt-3">
          <summary className="cursor-pointer text-gray-800 focus:outline-none focus:ring-2 focus:ring-indigo-500">
            Commands to run on the robot
          </summary>
          <dl className="mt-2 space-y-3 text-gray-700">
            {[
              ["First install of the agent", `sudo ./install.sh --provision-file ${fileName}`],
              ["Robot already has the agent", `sudo ./provision.sh ${fileName} --remove`],
            ].map(([label, command]) => (
              <div key={label}>
                <dt className="text-xs font-medium text-gray-600">{label}</dt>
                <dd className="mt-1 flex gap-2">
                  <code className="block min-w-0 flex-1 break-all rounded bg-white px-2 py-1 font-mono text-xs">{command}</code>
                  <button type="button" onClick={() => copy(command, command)} className="shrink-0 rounded border border-gray-300 px-2 text-xs font-semibold text-gray-700 hover:bg-white focus:outline-none focus:ring-2 focus:ring-indigo-500">
                    {copied === command ? "Copied" : "Copy"}
                  </button>
                </dd>
              </div>
            ))}
          </dl>
          <p className="mt-2 text-xs text-gray-600">
            Both update the agent and <span className="font-mono">bloom_node</span>'s <span className="font-mono">.env</span>,
            all-or-nothing, and restart them.
          </p>
        </details>
      </div>

      <span role="status" className="sr-only">
        {copied ? "Copied to clipboard" : ""}
      </span>
      <p role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
        {error}
      </p>

      <label className="flex items-start gap-2 text-sm text-gray-800">
        <input
          type="checkbox"
          checked={saved}
          onChange={(e) => onSavedChange(e.target.checked)}
          className="mt-0.5 h-4 w-4 rounded border-gray-300 text-indigo-600 focus:ring-indigo-500"
        />
        I've saved the secret. I understand it can't be shown again.
      </label>
    </div>
  );
}
