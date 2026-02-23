import { useState } from "react";
import { Link, useNavigate } from "react-router-dom";
import { EyeSlashIcon, EyeIcon } from "@heroicons/react/24/solid";
import { useApiClient } from "../context/ApiClientContext";
import { signInSession, dashboardPathForRole } from "../utils/auth";

export default function SignIn() {
  const [passwordShown, setPasswordShown] = useState(false);
  const [email, setEmail] = useState("");
  const [password, setPassword] = useState("");
  const [err, setErr] = useState("");
  const [submitting, setSubmitting] = useState(false);

  const navigate = useNavigate();
  const api = useApiClient();

  async function handleSubmit(e) {
    e.preventDefault();
    setErr("");
    setSubmitting(true);

    try {
      const data = await api.signIn(email, password);
      console.log("Login response:", data);
      const loginUser = data?.user;
      if (!loginUser) {
        throw new Error("Unexpected login response from server");
      }

      const userId = loginUser.id;
      let userEmail = loginUser.email;
      let fullName = loginUser.fullName;
      let role = loginUser.role;

      if (!role && userId) {
        const profile = await api.getUserProfile(userId);
        console.log("Profile response:", profile);
        role = profile?.role;
        fullName = profile?.fullName ?? fullName;
        userEmail = profile?.email ?? userEmail;
      }

      if (!role) {
        throw new Error("Login succeeded but no role was provided");
      }

      const user = {
        email: userEmail,
        fullName: fullName ?? "",
        role: role,
      };

      signInSession(user);

      const normalizedRole = role.toLowerCase();

      console.log("user.role:", user.role);
      console.log("normalizedRole:", normalizedRole);
      navigate(dashboardPathForRole(normalizedRole), { replace: true });
    } catch (error) {
      console.error("Sign in error:", error);
      if (error.message.includes("Invalid login attempt")) {
        setErr("Invalid email or password");
      } else if (error.message.includes("HTTP 401")) {
        setErr("Invalid email or password");
      } else {
        setErr("Sign in failed. Please try again.");
      }
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div className="min-h-screen bg-white text-gray-900 flex flex-col px-6 py-12 lg:px-8 mt-[100px] ">
      <div className="sm:mx-auto sm:w-full sm:max-w-sm">
        <h2 className="mt-10 text-center text-2xl font-bold tracking-tight">
          Welcome back
        </h2>
        <p className="text-center">
          Sign in to your Bloom account
        </p>
      </div>

      <div className="mt-10 sm:mx-auto sm:w-full sm:max-w-sm">
        <form onSubmit={handleSubmit} className="space-y-6">
          {err && <p className="text-sm text-red-600">{err}</p>}

          {/* Email */}
          <div>
            <label htmlFor="email" className="block text-sm font-medium text-gray-900">
              Email address
            </label>
            <div className="mt-2">
              <input
                id="email"
                name="email"
                type="email"
                autoComplete="email"
                required
                value={email}
                onChange={(e) => setEmail(e.target.value)}
                className="block w-full rounded-md bg-white px-3 py-1.5 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm autofill:bg-white"
                placeholder="name@mail.com"
              />
            </div>
          </div>

          {/* Password with eye toggle */}
          <div>
            <div className="flex items-center justify-between">
              <label htmlFor="password" className="block text-sm font-medium text-gray-900">
                Password
              </label>
              <div className="text-sm">
                <Link
                  to="/forgot-password"
                  className="font-semibold text-indigo-600 hover:text-indigo-500 no-underline"
                >
                  Forgot password?
                </Link>
              </div>
            </div>
            <div className="mt-2 relative">
              <input
                id="password"
                name="password"
                type={passwordShown ? "text" : "password"}
                autoComplete="current-password"
                required
                value={password}
                onChange={(e) => setPassword(e.target.value)}
                className="block w-full rounded-md bg-white px-3 py-1.5 pr-10 text-base text-gray-900 outline-1 -outline-offset-1 outline-gray-300 placeholder:text-gray-400 focus:outline-2 focus:-outline-offset-2 focus:outline-indigo-600 sm:text-sm autofill:bg-white"
                placeholder="********"
              />
              <button
                type="button"
                onClick={() => setPasswordShown((v) => !v)}
                className="absolute inset-y-0 right-0 flex items-center px-3 border-l border-black text-gray-400 hover:bg-g  hover:text-gray-700 rounded-r-md transition"
                aria-label="Toggle password visibility"
              >
                {passwordShown ? <EyeSlashIcon className="h-5 w-5 text-gray-500" /> : <EyeIcon className="h-5 w-5 text-gray-500" />}
              </button>
            </div>
          </div>

          {/* Submit */}
          <div>
            <button
              type="submit"
              disabled={submitting}
              className="flex w-full justify-center rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600 disabled:opacity-50"
            >
              {submitting ? "Signing in..." : "Sign in"}
            </button>
          </div>
        </form>
        <p className="mt-10 text-center text-sm text-gray-500">
          New member?{" "}
          <Link to="/sign-up" className="font-semibold text-indigo-600 hover:text-indigo-500 no-underline">
            Sign up
          </Link>
        </p>
      </div>
    </div>
  );
}
