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

  // Build breadcrumbs from pathname: /dashboard/admin -> Home / Dashboard / Admin
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
    <div className="min-h-screen bg-white text-gray-900">
      <div className="mx-auto max-w-6xl px-6 py-10 lg:px-8 mt-[100px]">
        {/* Header */}
        <header className="mb-6">
          <div className="flex items-start justify-between gap-4">
            <div className="space-y-2">
              <h1 className="text-2xl font-bold tracking-tight">{title}</h1>

              {/* Breadcrumbs (no bullets, inline) */}
              <nav aria-label="Breadcrumb">
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
              </nav>

              {/* Section nav (pills, no bullets) */}
              <ul className="m-0 mt-3 flex list-none flex-wrap gap-2 p-0">
                {nav.map((item) => (
                  <li key={item.to}>
                    <NavLink
                      to={item.to}
                      className={({ isActive }) =>
                        cls(
                          "inline-flex items-center rounded-md px-3 py-1.5 text-sm font-medium shadow-xs ring-1 ring-inset",
                          isActive
                            ? "bg-indigo-600 text-white ring-indigo-600"
                            : "bg-white text-indigo-700 ring-indigo-200 hover:bg-indigo-50"
                        )
                      }
                    >
                      {item.label}
                    </NavLink>
                  </li>
                ))}
              </ul>
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
        <main className="rounded-2xl bg-gray-50 p-6 shadow-sm ring-1 ring-gray-100">
          {children}
        </main>
      </div>
    </div>
  );
}
