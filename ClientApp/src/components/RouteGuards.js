import { Navigate, Outlet } from "react-router-dom";
import { getSession } from "../utils/auth";

export function RequireAuth() {
  const session = getSession();
  if (!session) return <Navigate to="/sign-in" replace />;
  return <Outlet />;
}

export function RequireRole({ allow }) {
  const session = getSession();
  if (!session) return <Navigate to="/sign-in" replace />;

  const role = session.role?.toLowerCase();
  const allowed = allow.map(r => r.toLowerCase());

  if (!allowed.includes(role)) return <Navigate to="/" replace />;
  return <Outlet />;
}
