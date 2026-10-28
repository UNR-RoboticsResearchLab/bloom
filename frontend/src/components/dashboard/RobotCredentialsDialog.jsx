// RobotCredentialsDialog.jsx
// Issue a robot new credentials, one of two ways:
//   push  -- the backend sends the new secret to the robot's agent through its live connection and the
//            agent applies it (agent + bloom_node). The old secret keeps working until the robot uses
//            the new one, so a failed push can't lock the robot out. The secret never reaches the browser.
//   file  -- the old secret stops working immediately; the new one is shown once, with a provisioning
//            file to carry to the robot. For offline robots or when a push fails.

import React, { useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import SecretReveal from "./SecretReveal";
import { apiErrorMessage } from "../../utils/provisioning";

const primaryButton =
  "rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300";
const secondaryButton =
  "rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60";

export default function RobotCredentialsDialog({ robot, onClose, onChanged }) {
  const api = useApiClient();
  const id = useId();
  const canPush = !!robot.online && robot.provisioned !== false;

  const [method, setMethod] = useState(canPush ? "push" : "file");
  const [step, setStep] = useState("choose"); // choose | pushed | secret
  const [working, setWorking] = useState(false);
  const [error, setError] = useState("");
  const [secret, setSecret] = useState("");
  const [saved, setSaved] = useState(false);

  const locked = step === "secret" && !saved;
  const changed = step !== "choose";

  function close() {
    if (changed) onChanged?.();
    onClose();
  }

  async function submit(e) {
    e.preventDefault();
    setError("");
    setWorking(true);
    try {
      if (method === "push") {
        await api.pushRobotCredentials(robot.id);
        setStep("pushed");
      } else {
        const res = await api.rotateRobotCredentials(robot.id);
        if (!res?.secret) throw new Error("Unexpected response from the server.");
        setSecret(res.secret);
        setStep("secret");
      }
    } catch (err) {
      console.error("Failed to issue credentials:", err);
      setError(
        apiErrorMessage(err, "Couldn't issue credentials. Check your connection and try again.") +
          (method === "push" ? " You can download a provisioning file instead." : "")
      );
    } finally {
      setWorking(false);
    }
  }

  const titles = { choose: "Issue new credentials", pushed: "Credentials delivered", secret: "New credentials" };
  const descriptions = {
    choose: `Gives ${robot.name} a new secret.`,
    pushed: `${robot.name} applied the new secret and is reconnecting.`,
    secret: "Save this secret now. It is shown once and can't be retrieved later.",
  };

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={close} dismissible={!locked && !working}>
      <div className="flex items-start justify-between gap-4 border-b border-gray-200 px-6 py-4">
        <div>
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">
            {titles[step]}
          </h2>
          <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">
            {descriptions[step]}
          </p>
        </div>
        {!locked && (
          <button
            type="button"
            onClick={close}
            aria-label="Close"
            className="rounded text-xl leading-none text-gray-400 hover:text-gray-600 focus:outline-none focus:ring-2 focus:ring-indigo-500"
          >
            &times;
          </button>
        )}
      </div>

      {step === "choose" && (
        <form onSubmit={submit} className="space-y-4 px-6 py-5">
          <fieldset className="space-y-3">
            <legend className="text-sm font-medium text-gray-900">How should the robot get it?</legend>

            <label className={`flex items-start gap-3 rounded-lg border p-3 text-sm ${canPush ? "cursor-pointer border-gray-200" : "border-gray-100 bg-gray-50 text-gray-400"}`}>
              <input
                type="radio"
                name={`${id}-method`}
                value="push"
                checked={method === "push"}
                onChange={() => setMethod("push")}
                disabled={!canPush}
                data-autofocus={canPush ? true : undefined}
                className="mt-1 h-4 w-4 text-indigo-600 focus:ring-indigo-500"
              />
              <span>
                <span className="block font-medium text-gray-900">Push to the robot now</span>
                <span className="block text-gray-600">
                  {canPush
                    ? "Sent through its live connection and applied automatically. The old secret keeps working until the robot uses the new one, so this can't lock it out."
                    : robot.provisioned === false
                    ? "Unavailable: this robot has no credentials yet."
                    : "Unavailable: the robot is offline."}
                </span>
              </span>
            </label>

            <label className="flex cursor-pointer items-start gap-3 rounded-lg border border-gray-200 p-3 text-sm">
              <input
                type="radio"
                name={`${id}-method`}
                value="file"
                checked={method === "file"}
                onChange={() => setMethod("file")}
                data-autofocus={canPush ? undefined : true}
                className="mt-1 h-4 w-4 text-indigo-600 focus:ring-indigo-500"
              />
              <span>
                <span className="block font-medium text-gray-900">Download a provisioning file</span>
                <span className="block text-gray-600">
                  For offline robots or if a push fails. <strong>The old secret stops working immediately</strong>,
                  and so does the robot's connection and any open shell, until you apply the file on the robot.
                </span>
              </span>
            </label>
          </fieldset>

          <p id={`${id}-error`} role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>
            {error}
          </p>

          <div className="flex justify-end gap-2 pt-1">
            <button type="button" onClick={close} disabled={working} className={secondaryButton}>
              Cancel
            </button>
            <button type="submit" disabled={working} className={primaryButton}>
              {working ? "Working…" : method === "push" ? "Push credentials" : "Issue credentials"}
            </button>
          </div>
        </form>
      )}

      {step === "pushed" && (
        <div className="space-y-4 px-6 py-5">
          <p className="text-sm text-gray-700">
            The new secret was applied to the agent and <span className="font-mono">bloom_node</span> on the
            robot. It briefly drops offline while it reconnects; any open shell on it ended.
          </p>
          <div className="flex justify-end">
            <button type="button" onClick={close} data-autofocus className={primaryButton}>
              Done
            </button>
          </div>
        </div>
      )}

      {step === "secret" && (
        <div className="space-y-4 px-6 py-5">
          <SecretReveal robot={robot} secret={secret} saved={saved} onSavedChange={setSaved} />
          <div className="flex justify-end pt-1">
            <button type="button" onClick={close} disabled={!saved} className={primaryButton}>
              Done
            </button>
          </div>
        </div>
      )}
    </Modal>
  );
}
