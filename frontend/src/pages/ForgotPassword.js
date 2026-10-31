import { useState } from "react";
import { Link } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";

export default function ForgotPassword() {
  const [email, setEmail] = useState("");
  const [err, setErr] = useState("");
  const [sent, setSent] = useState(false);
  const [submitting, setSubmitting] = useState(false);

  const api = useApiClient();

  async function handleSubmit(e) {
    e.preventDefault();
    setErr("");
    setSubmitting(true);

    try {
      await api.forgotPassword(email);
      setSent(true);
    } catch (error) {
      console.error("Forgot password error:", error);
      if (error.message.includes("HTTP 429")) {
        setErr("Too many requests. Please wait a few minutes and try again.");
      } else {
        setErr("Something went wrong. Please try again.");
      }
    } finally {
      setSubmitting(false);
    }
  }

  return (
    <div className="min-h-screen bg-white text-gray-900 flex flex-col px-6 py-12 lg:px-8 mt-[100px]">
      <div className="sm:mx-auto sm:w-full sm:max-w-sm">
        <h2 className="mt-10 text-center text-2xl font-bold tracking-tight">Reset your password</h2>
        {err && (
          <p className="mt-4 text-center text-sm text-red-600" role="alert">
            {err}
          </p>
        )}
      </div>

      <div className="mt-10 sm:mx-auto sm:w-full sm:max-w-sm">
        {sent ? (
          <p className="text-center text-sm text-gray-700">
            If that address belongs to an account, we sent an email with instructions. The link
            expires in 24 hours.
          </p>
        ) : (
          <form onSubmit={handleSubmit} className="space-y-6">
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
            <button
              type="submit"
              disabled={submitting}
              className="flex w-full justify-center rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500 disabled:opacity-50"
            >
              {submitting ? "Sending..." : "Send reset link"}
            </button>
          </form>
        )}
        <p className="mt-6 text-center text-sm">
          <Link to="/sign-in" className="font-semibold text-indigo-600 hover:text-indigo-500 no-underline">
            Back to sign in
          </Link>
        </p>
      </div>
    </div>
  );
}
