import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";
import { getSession, updateSession, signOut, dashboardPathForRole } from "../utils/auth";
import DashboardLayout from "../components/dashboard/DashboardLayout";

export default function EditAccount() {
  const api = useApiClient();
  const navigate = useNavigate();
  const session = getSession();

  const [fullName, setFullName] = useState("");
  const [userName, setUserName] = useState("");
  const [email, setEmail] = useState("");
  const [loading, setLoading] = useState(true);
  const [submitting, setSubmitting] = useState(false);
  const [deleting, setDeleting] = useState(false);
  const [confirmDelete, setConfirmDelete] = useState(false);
  const [err, setErr] = useState("");
  const [success, setSuccess] = useState("");

  useEffect(() => {
    if (!session?.id) {
      navigate("/sign-in", { replace: true });
      return;
    }

    let mounted = true;
    async function load() {
      try {
        const profile = await api.getUserProfile(session.id);
        if (!mounted) return;
        setFullName(profile?.fullName ?? "");
        setUserName(profile?.userName ?? "");
        setEmail(profile?.email ?? "");
      } catch (e) {
        console.error("Failed to load profile:", e);
        if (mounted) setErr("Could not load your profile.");
      } finally {
        if (mounted) setLoading(false);
      }
    }
    load();
    return () => {
      mounted = false;
    };
  }, [api, session?.id, navigate]);

  async function handleSubmit(e) {
    e.preventDefault();
    setErr("");
    setSuccess("");
    setSubmitting(true);

    try {
      const res = await api.updateUserProfile(session.id, {
        fullName,
        userName,
        email,
      });
      const updated = res?.user ?? res?.User ?? null;
      updateSession({
        email: updated?.email ?? email,
        fullName: updated?.fullName ?? fullName,
        userName: updated?.userName ?? userName,
        name: updated?.fullName ?? fullName,
      });
      setSuccess("Account updated successfully.");
    } catch (error) {
      console.error("Update failed:", error);
      setErr("Failed to update account. Please try again.");
    } finally {
      setSubmitting(false);
    }
  }

  async function handleDelete() {
    setErr("");
    setSuccess("");
    setDeleting(true);
    try {
      await api.deleteUserProfile(session.id);
      signOut();
      navigate("/sign-in", { replace: true });
    } catch (error) {
      console.error("Delete failed:", error);
      setErr("Failed to delete account.");
      setDeleting(false);
      setConfirmDelete(false);
    }
  }

  const dashboardPath = dashboardPathForRole(session?.role);

  return (
    <DashboardLayout title="Edit Account">
      <div className="mx-auto max-w-xl">
        <section className="rounded-lg border border-gray-200 bg-white p-6">
          {loading ? (
            <p className="text-sm text-gray-500">Loading...</p>
          ) : (
            <form onSubmit={handleSubmit} className="space-y-5">
              {err && <p className="text-sm text-red-600">{err}</p>}
              {success && <p className="text-sm text-green-600">{success}</p>}

              <div>
                <label htmlFor="fullName" className="block text-sm font-medium text-gray-900">
                  Full name
                </label>
                <div className="mt-2">
                  <input
                    id="fullName"
                    name="fullName"
                    type="text"
                    value={fullName}
                    onChange={(e) => setFullName(e.target.value)}
                    className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                  />
                </div>
              </div>

              <div>
                <label htmlFor="userName" className="block text-sm font-medium text-gray-900">
                  Username
                </label>
                <div className="mt-2">
                  <input
                    id="userName"
                    name="userName"
                    type="text"
                    value={userName}
                    onChange={(e) => setUserName(e.target.value)}
                    className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                  />
                </div>
              </div>

              <div>
                <label htmlFor="email" className="block text-sm font-medium text-gray-900">
                  Email
                </label>
                <div className="mt-2">
                  <input
                    id="email"
                    name="email"
                    type="email"
                    value={email}
                    onChange={(e) => setEmail(e.target.value)}
                    className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm"
                  />
                </div>
              </div>

              <div className="flex items-center justify-between gap-3 pt-2">
                <button
                  type="button"
                  onClick={() => navigate(dashboardPath)}
                  className="rounded-md border border-gray-300 px-3 py-1.5 text-sm font-semibold text-gray-700 hover:bg-gray-50"
                >
                  Cancel
                </button>
                <button
                  type="submit"
                  disabled={submitting}
                  className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600 disabled:opacity-50"
                >
                  {submitting ? "Saving..." : "Save changes"}
                </button>
              </div>
            </form>
          )}
        </section>

        <section className="mt-6 rounded-lg border border-red-200 bg-white p-6">
          <h3 className="text-base font-semibold text-red-700">Danger zone</h3>
          <p className="mt-1 text-sm text-gray-600">
            Permanently delete your account. This action cannot be undone.
          </p>

          {!confirmDelete ? (
            <button
              type="button"
              onClick={() => setConfirmDelete(true)}
              className="mt-4 rounded-md bg-red-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-red-500"
            >
              Delete account
            </button>
          ) : (
            <div className="mt-4 flex items-center gap-3">
              <span className="text-sm text-gray-700">Are you sure?</span>
              <button
                type="button"
                onClick={handleDelete}
                disabled={deleting}
                className="rounded-md bg-red-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-red-500 disabled:opacity-50"
              >
                {deleting ? "Deleting..." : "Yes, delete"}
              </button>
              <button
                type="button"
                onClick={() => setConfirmDelete(false)}
                disabled={deleting}
                className="rounded-md border border-gray-300 px-3 py-1.5 text-sm font-semibold text-gray-700 hover:bg-gray-50"
              >
                Cancel
              </button>
            </div>
          )}
        </section>
      </div>
    </DashboardLayout>
  );
}
