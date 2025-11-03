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
  return users.find(
    u => u.email.toLowerCase() === email.toLowerCase() && u.password === password
  ) || null;
}

export function signInSession(user) {
  localStorage.setItem("authUser", JSON.stringify({ email: user.email, role: user.role, name: user.fullName || "" }));
}

export function getSession() {
  try { return JSON.parse(localStorage.getItem("authUser")); } catch { return null; }
}

export function signOut() {
  localStorage.removeItem("authUser");
}

export function dashboardPathForRole(role) {
  switch (role) {
    case "admin":   return "/dashboard/admin";
    case "teacher": return "/dashboard/teacher";
    case "student": return "/dashboard/student";
    case "slp":     return "/dashboard/slp";
    default:        return "/sign-in";
  }
}
