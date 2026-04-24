import { dashboardPathForRole } from "./auth";

describe("dashboardPathForRole", () => {
  // Tests to verify that users are routed to the correct dashboard based on their role
  test("routes roles to the correct dashboard path", () => {
    expect(dashboardPathForRole("admin")).toBe("/dashboard/admin");
    expect(dashboardPathForRole("teacher")).toBe("/dashboard/teacher");
    expect(dashboardPathForRole("student")).toBe("/dashboard/student");
    expect(dashboardPathForRole("slp")).toBe("/dashboard/slp");
  });

  // Tests to ensure that the function handles edge cases, such as unknown roles or null values
  test("handles case with differences and a missing role", () => {
    //Pass
    expect(dashboardPathForRole("TeAcHeR")).toBe("/dashboard/teacher");
    //Fail
    expect(dashboardPathForRole("unknown")).toBe("/sign-in");
    expect(dashboardPathForRole(null)).toBe("/sign-in");
  });
});









