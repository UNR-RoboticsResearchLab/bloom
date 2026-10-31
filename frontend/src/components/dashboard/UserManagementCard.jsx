// UserManagementCard.jsx
// Admin user and role management: every account with its role, a role picker, delete, and an
// Add user button. Backed by /api/admin/users (Admin only). Changing a role or deleting asks for
// confirmation, and the backend refuses self-changes and removing the last Admin; its message
// is shown in the dialog.

import React, { useCallback, useEffect, useId, useMemo, useState } from "react";
import { useApiClient } from "../../context/ApiClientContext";
import Modal from "../ui/Modal";
import AddUserDialog from "./AddUserDialog";
import { apiErrorStatus } from "../../utils/provisioning";

// ApiClient.request throws `HTTP <status>: <body>`; the backend's errors are {"message": "..."}.
export function apiErrorMessage(err, fallback) {
  const body = (err?.message || "").replace(/^HTTP \d{3}:\s*/, "");
  try {
    const parsed = JSON.parse(body);
    return parsed.message || parsed.Message || fallback;
  } catch {
    return fallback;
  }
}

function ConfirmDialog({ title, description, confirmLabel, busyLabel, destructive, onConfirm, onClose }) {
  const id = useId();
  const [busy, setBusy] = useState(false);
  const [error, setError] = useState("");

  async function handleSubmit(e) {
    e.preventDefault();
    setError("");
    setBusy(true);
    try {
      await onConfirm();
      onClose();
    } catch (err) {
      console.error("User action failed:", err);
      setError(
        [401, 403].includes(apiErrorStatus(err))
          ? "You don't have permission to do that."
          : apiErrorMessage(err, "That didn't work. Refresh and try again.")
      );
      setBusy(false);
    }
  }

  return (
    <Modal titleId={`${id}-title`} descriptionId={`${id}-desc`} onClose={onClose} dismissible={!busy}>
      <form onSubmit={handleSubmit}>
        <div className="border-b border-gray-200 px-6 py-4">
          <h2 id={`${id}-title`} className="text-lg font-semibold text-gray-900">{title}</h2>
          <p id={`${id}-desc`} className="mt-1 text-sm text-gray-600">{description}</p>
        </div>
        <div className="space-y-4 px-6 py-5">
          <p role="alert" className={error ? "text-sm text-red-600" : "sr-only"}>{error}</p>
          <div className="flex justify-end gap-2">
            <button
              type="button"
              onClick={onClose}
              disabled={busy}
              data-autofocus
              className="rounded-md border border-gray-300 px-4 py-2 text-sm font-semibold text-gray-700 hover:bg-gray-50 focus:outline-none focus:ring-2 focus:ring-indigo-500 disabled:opacity-60"
            >
              Cancel
            </button>
            <button
              type="submit"
              disabled={busy}
              className={[
                "rounded-md px-4 py-2 text-sm font-semibold text-white focus:outline-none focus:ring-2 focus:ring-offset-1 disabled:cursor-not-allowed disabled:bg-gray-300",
                destructive ? "bg-red-600 hover:bg-red-500 focus:ring-red-500" : "bg-indigo-600 hover:bg-indigo-500 focus:ring-indigo-500",
              ].join(" ")}
            >
              {busy ? busyLabel : confirmLabel}
            </button>
          </div>
        </div>
      </form>
    </Modal>
  );
}

export default function UserManagementCard({ onNotice }) {
  const api = useApiClient();
  const [users, setUsers] = useState(null);
  const [roles, setRoles] = useState([]);
  const [meId, setMeId] = useState(null);
  const [error, setError] = useState("");
  const [query, setQuery] = useState("");
  const [roleChange, setRoleChange] = useState(null); // { user, role }
  const [deleting, setDeleting] = useState(null);
  const [adding, setAdding] = useState(false);

  const load = useCallback(async () => {
    try {
      setUsers(await api.getAdminUsers());
      setError("");
    } catch (err) {
      console.error("failed to load users:", err);
      setError("Couldn't load users.");
    }
  }, [api]);

  useEffect(() => {
    load();
    api.getAssignableRoles().then(setRoles).catch((err) => console.error("failed to load roles:", err));
    api.getMe().then((me) => setMeId(me?.id ?? me?.Id ?? null)).catch(() => {});
  }, [api, load]);

  const shown = useMemo(() => {
    const q = query.trim().toLowerCase();
    if (!q) return users || [];
    return (users || []).filter((u) =>
      [u.fullName, u.email, u.userName, u.role].some((v) => (v || "").toLowerCase().includes(q))
    );
  }, [users, query]);

  return (
    <section className="mt-6 rounded-lg border border-gray-200 bg-white p-4">
      <div className="mb-4 flex flex-wrap items-baseline justify-between gap-4">
        <div>
          <h3 className="text-base font-semibold">Users and roles</h3>
          <p className="mt-1 text-sm text-gray-600">
            Accounts and what each can do. Admins get fleet access and remote shell on every robot.
          </p>
        </div>
        <div className="flex items-center gap-3">
          <input
            type="search"
            value={query}
            onChange={(e) => setQuery(e.target.value)}
            placeholder="Search users"
            aria-label="Search users"
            className="rounded-md border border-gray-300 px-3 py-1.5 text-sm shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
          />
          <button
            type="button"
            onClick={() => setAdding(true)}
            className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1"
          >
            Add user
          </button>
        </div>
      </div>

      {error && <p className="mb-3 text-sm text-red-600">{error}</p>}

      {users === null && !error ? (
        <p className="text-sm text-gray-500">Loading…</p>
      ) : shown.length === 0 && !error ? (
        <p className="text-sm text-gray-500">{query ? "No users match." : "No users."}</p>
      ) : (
        <div className="max-h-[480px] overflow-auto rounded-lg border border-gray-200">
          <table className="w-full text-left text-sm">
            <thead className="sticky top-0 bg-gray-50 text-xs uppercase tracking-wide text-gray-500">
              <tr>
                <th className="px-3 py-2 font-medium">Name</th>
                <th className="px-3 py-2 font-medium">Email</th>
                <th className="px-3 py-2 font-medium">Role</th>
                <th className="px-3 py-2 font-medium">Created</th>
                <th className="px-3 py-2" />
              </tr>
            </thead>
            <tbody className="divide-y divide-gray-100">
              {shown.map((u) => {
                const isMe = u.id === meId;
                const label = u.fullName || u.email;
                // Roles outside the assignable list (e.g. Participant) are shown but not editable.
                const editable = !isMe && roles.includes(u.role);
                return (
                  <tr key={u.id}>
                    <td className="px-3 py-2 text-gray-900">
                      {u.fullName || "—"}
                      {isMe && <span className="ml-2 rounded-full bg-gray-100 px-2 py-0.5 text-xs text-gray-600">you</span>}
                    </td>
                    <td className="px-3 py-2 text-gray-700">{u.email}</td>
                    <td className="px-3 py-2">
                      {editable ? (
                        <select
                          value={u.role}
                          onChange={(e) => e.target.value !== u.role && setRoleChange({ user: u, role: e.target.value })}
                          aria-label={`Role for ${label}`}
                          className="rounded-md border border-gray-300 bg-white px-2 py-1 text-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
                        >
                          {roles.map((r) => (
                            <option key={r} value={r}>{r}</option>
                          ))}
                        </select>
                      ) : (
                        <span className="text-gray-700">{u.role || "—"}</span>
                      )}
                    </td>
                    <td className="whitespace-nowrap px-3 py-2 text-gray-500">
                      {u.createdDate ? new Date(u.createdDate).toLocaleDateString() : "—"}
                    </td>
                    <td className="px-3 py-2 text-right">
                      <button
                        type="button"
                        onClick={() => setDeleting(u)}
                        disabled={isMe}
                        aria-label={`Delete ${label}`}
                        title={isMe ? "You can't delete your own account" : undefined}
                        className="rounded-md border border-red-200 bg-white px-2.5 py-1 text-sm font-semibold text-red-700 hover:bg-red-50 focus:outline-none focus:ring-2 focus:ring-red-500 disabled:cursor-not-allowed disabled:border-gray-200 disabled:text-gray-400 disabled:hover:bg-white"
                      >
                        Delete
                      </button>
                    </td>
                  </tr>
                );
              })}
            </tbody>
          </table>
        </div>
      )}

      {roleChange && (
        <ConfirmDialog
          key={roleChange.user.id}
          title={`Make ${roleChange.user.fullName || roleChange.user.email} ${roleChange.role}?`}
          description={
            roleChange.role === "Admin"
              ? "Admins can manage the fleet, open a remote shell on any robot, and manage every account."
              : roleChange.user.role === "Admin"
              ? "This removes their administrator access."
              : `Their role changes from ${roleChange.user.role} to ${roleChange.role}.`
          }
          confirmLabel="Change role"
          busyLabel="Changing…"
          destructive={roleChange.role === "Admin" || roleChange.user.role === "Admin"}
          onConfirm={async () => {
            await api.setUserRole(roleChange.user.id, roleChange.role);
            onNotice?.(`${roleChange.user.fullName || roleChange.user.email} is now ${roleChange.role}.`);
            await load();
          }}
          onClose={() => setRoleChange(null)}
        />
      )}

      {deleting && (
        <ConfirmDialog
          key={deleting.id}
          title={`Delete ${deleting.fullName || deleting.email}?`}
          description="This permanently removes the account and its student links. It can't be undone."
          confirmLabel="Delete user"
          busyLabel="Deleting…"
          destructive
          onConfirm={async () => {
            await api.deleteAdminUser(deleting.id);
            onNotice?.(`Deleted ${deleting.fullName || deleting.email}.`);
            await load();
          }}
          onClose={() => setDeleting(null)}
        />
      )}

      {adding && (
        <AddUserDialog
          roles={roles}
          onClose={() => setAdding(false)}
          onCreated={(u) => {
            onNotice?.(`Created ${u.fullName || u.email}.`);
            load();
          }}
        />
      )}
    </section>
  );
}
