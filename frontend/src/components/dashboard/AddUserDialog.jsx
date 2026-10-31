// AddUserDialog.jsx
// Admin creates an account with any assignable role (POST /api/user/create, which lets an Admin
// create other Admins). The password is chosen by the admin and not shown again.

import React, { useId, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import { apiErrorMessage } from "./UserManagementCard";

const INPUT =
  "mt-1 block w-full rounded-md border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200";

export default function AddUserDialog({ roles, onClose, onCreated }) {
  const api = useApiClient();
  const id = useId();
  const [fullName, setFullName] = useState("");
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [role, setRole] = useState("SLP");
  const [submitting, setSubmitting] = useState(false);
  const [error, setError] = useState("");

  async function handleSubmit(e) {
    e.preventDefault();
    setError("");
    setSubmitting(true);
    try {
      await api.signUp({ fullName: fullName.trim(), email: email.trim(), password, role });
      onCreated?.({ fullName: fullName.trim(), email: email.trim() });
      onClose();
    } catch (err) {
      console.error("Failed to create user:", err);
      setError(apiErrorMessage(err, "Couldn't create the user."));
      setSubmitting(false);
    }
  }

  return (
    <Modal titleId={`${id}-title`} onClose={onClose} dismissible={!submitting}>
      <form onSubmit={handleSubmit}>
        <div className="border-b border-gray-200 px-6 py-4">
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">Add user</h2>
        </div>
        <div className="space-y-4 px-6 py-5">
          <div>
            <label htmlFor={`${id}-name`} className="block text-sm font-medium text-gray-900">Full name</label>
            <input id={`${id}-name`} data-autofocus required value={fullName} onChange={(e) => setFullName(e.target.value)} className={INPUT} />
          </div>
          <div>
            <label htmlFor={`${id}-email`} className="block text-sm font-medium text-gray-900">Email</label>
            <input id={`${id}-email`} type="email" required value={email} onChange={(e) => setEmail(e.target.value)} autoComplete="off" className={INPUT} />
          </div>
          <div>
            <label htmlFor={`${id}-password`} className="block text-sm font-medium text-gray-900">Temporary password</label>
            <input id={`${id}-password`} type="password" required value={password} onChange={(e) => setPassword(e.target.value)} autoComplete="new-password" className={INPUT} />
          </div>
          <div>
            <label htmlFor={`${id}-role`} className="block text-sm font-medium text-gray-900">Role</label>
            <select id={`${id}-role`} value={role} onChange={(e) => setRole(e.target.value)} className={INPUT}>
              {(roles.length ? roles : ["SLP"]).map((r) => (
                <option key={r} value={r}>{r}</option>
              ))}
            </select>
            {role === "Admin" && (
              <p className="mt-1 text-xs text-amber-700">Admins can open a remote shell on every robot.</p>
            )}
          </div>

          <p role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>{error}</p>

          <div className="flex justify-end gap-2 pt-1">
            <button type="button" onClick={onClose} disabled={submitting} className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60">
              Cancel
            </button>
            <button type="submit" disabled={submitting} className="rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300">
              {submitting ? "Creating…" : "Create user"}
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
}
