export function getUsers() {
  try { return JSON.parse(localStorage.getItem("users")) || []; } catch { return []; }
}

export function saveUser(user) {
  const users = getUsers();
  const exists = users.some(u => u.email.toLowerCase() === user.email.toLowerCase());
  if (!exists) {
    users.push(user);
    localStorage.setItem("users", JSON.stringify(users));
  }
}

export function findUser(email, password) {
  const users = getUsers();
  return (
    users.find(
      u =>
        u.email.toLowerCase() === email.toLowerCase() &&
        u.password === password
    ) || null
  );
}

export function signInSession(user) {
  localStorage.setItem(
    "authUser",
    JSON.stringify({
      id: user.id,
      email: user.email,
      role: user.role,
      name: user.fullName || "",
      fullName: user.fullName || "",
      userName: user.userName || "",
    })
  );
}

export function updateSession(partial) {
  const current = getSession() || {};
  const next = { ...current, ...partial };
  localStorage.setItem("authUser", JSON.stringify(next));
  return next;
}

export function getSession() {
  try {
    return JSON.parse(localStorage.getItem("authUser"));
  } catch {
    return null;
  }
}

export function signOut() {
  localStorage.removeItem("authUser");
}


// Returns the correct dashboard route based on user role
// Used after authentication for role-based navigation
export function dashboardPathForRole(role) {
  // If no role is provided, redirect to sign-in
  if (!role) return "/sign-in";

  // Normalize role to ensure consistent matching
  const normalized = role.toLowerCase();

  // Debug log to verify role mapping during development
  console.log("dashboardPathForRole called with role:", role);

  // Map each role to its corresponding dashboard route
  switch (normalized) {
    case "admin":
      return "/dashboard/admin";
    case "teacher":
      return "/dashboard/teacher";
    case "student":
      return "/dashboard/student";
    case "slp":
      return "/dashboard/slp";

    // Fallback for unknown roles  
    default:
      return "/sign-in";
  }
}
