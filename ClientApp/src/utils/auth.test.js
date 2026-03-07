// src/utils/auth.test.js
import { dashboardPathForRole } from "./auth";

describe("dashboardPathForRole", () => {
  test("routes roles to the correct dashboard path", () => {
    expect(dashboardPathForRole("admin")).toBe("/dashboard/admin");
    expect(dashboardPathForRole("teacher")).toBe("/dashboard/teacher");
    expect(dashboardPathForRole("student")).toBe("/dashboard/student");
    expect(dashboardPathForRole("slp")).toBe("/dashboard/slp");
  });

  test("handles case with differences and a missing role", () => {
    expect(dashboardPathForRole("TeAcHeR")).toBe("/dashboard/teacher");
    expect(dashboardPathForRole("unknown")).toBe("/sign-in");
    expect(dashboardPathForRole(null)).toBe("/sign-in");
  });
});