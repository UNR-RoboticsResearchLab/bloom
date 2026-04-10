// src/components/dashboard/DashboardLayout.js
import { Link, NavLink, useLocation } from "react-router-dom";
import { getSession, signOut } from "../../utils/auth";

function cls(...xs) {
  return xs.filter(Boolean).join(" ");
}

function titleCase(s = "") {
  return s
    .split("-")
    .map((w) => w.charAt(0).toUpperCase() + w.slice(1))
    .join(" ");
}

export default function DashboardLayout({ title, children, actions = null }) {
  const session = getSession();
  const location = useLocation();

  const parts = location.pathname.split("/").filter(Boolean);
  const crumbs = [{ to: "/", label: "Home" }].concat(
    parts.map((seg, i) => ({
      to: "/" + parts.slice(0, i + 1).join("/"),
      label: titleCase(seg),
    }))
  );

  const nav = [
    { to: "/", label: "Home" },
    { to: "/fetch-data", label: "Fetch Data" },
    { to: "/counter", label: "Counter" },
  ];

  const role = session?.role?.toUpperCase();

  return (
    <div className="rounded-lg border p-6 shadow-sm min-h-screen bg-white text-gray-900 mt-[50px]">
      <div className="mx-auto max-w-6xl px-6 py-10 lg:px-8">
        {/* Header */}
        <header className="mb-6">
          <div className="flex items-start justify-between gap-4">
            <div className="space-y-2">
              <h1 className="text-2xl font-bold tracking-tight">{title}</h1>

              {/* Breadcrumbs (no bullets, inline) */}
              {/* <nav aria-label="Breadcrumb">
                <ol className="m-0 flex list-none items-center gap-2 p-0 text-sm">
                  {crumbs.map((c, i) => (
                    <li key={c.to} className="flex items-center gap-2">
                      {i > 0 && <span className="text-gray-400">/</span>}
                      {i < crumbs.length - 1 ? (
                        <Link className="text-indigo-600 hover:text-indigo-500" to={c.to}>
                          {c.label}
                        </Link>
                      ) : (
                        <span className="text-gray-700">{c.label}</span>
                      )}
                    </li>
                  ))}
                </ol>
              </nav> */}

            </div>

            {/* Right side actions */}
            <div className="flex shrink-0 items-center gap-3">
              <div className="hidden sm:flex items-center gap-2 rounded-full border border-gray-200 bg-white px-3 py-1.5 shadow-sm">
                <span className="text-sm text-gray-700">{session?.email}</span>
                {role ? (
                  <span className="rounded-full bg-indigo-50 px-2 py-0.5 text-xs font-semibold text-indigo-700">
                    {role}
                  </span>
                ) : null}
              </div>

              {actions}

              <button
                onClick={() => {
                  signOut();
                  window.location.assign("/sign-in");
                }}
                className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
              >
                Sign out
              </button>
            </div>
          </div>
        </header>

        {/* Content card */}
        <main className="">
          {children}
        </main>
      </div>
    </div>
  );
}
