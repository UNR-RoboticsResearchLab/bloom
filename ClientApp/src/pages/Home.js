// src/pages/Home.js
import { Link, useNavigate } from "react-router-dom";
import { getSession, dashboardPathForRole } from "../utils/auth";

export default function Home() {
  const session = getSession();
  const navigate = useNavigate();

  function handleGetStarted() {
    if (session?.role) {
      navigate(dashboardPathForRole(session.role), { replace: true });
    } else {
      navigate("/sign-up");
    }
  }

  return (
    <div className="min-h-screen bg-white text-gray-900">
      <div className="mx-auto max-w-6xl px-6 py-10 lg:px-8 mt-[100px]">
        {/* Hero */}
        <section className="rounded-2xl bg-gray-50 p-8 shadow-sm ring-1 ring-gray-100">
          <div className="flex flex-col items-start gap-4 sm:flex-row sm:items-center sm:justify-between">
            <div>
              <h1 className="text-3xl font-bold tracking-tight">Bloom</h1>
              <p className="mt-2 max-w-2xl text-gray-700">
                A robotics assisted speech therapy platform that gives SLPs and teachers simple tools to run lessons, track progress, and help students practice with instant feedback. Bloom pairs lesson content with on-device and cloud components so you can choose the right setup for your classroom or clinic.
              </p>
            </div>
            <div className="flex gap-3">
              <button
                onClick={handleGetStarted}
                className="rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500 focus-visible:outline-2 focus-visible:outline-offset-2 focus-visible:outline-indigo-600"
              >
                {session?.role ? "Go to my dashboard" : "Get started"}
              </button>
              {!session?.role && (
                <Link
                  to="/sign-in"
                  className="rounded-md bg-white px-4 py-2 text-sm font-semibold text-indigo-700 ring-1 ring-inset ring-indigo-200 shadow-xs hover:bg-indigo-50"
                >
                  Sign in
                </Link>
              )}
            </div>
          </div>
        </section>

        {/* Quick role cards */}
        <section className="mt-8 grid gap-4 sm:grid-cols-2 lg:grid-cols-4">
          <RoleCard
            title="Admin"
            desc="See system health such as memory and audio latency, network quality, and error rates for STT and TTS."
            to="/dashboard/admin"
          />
          <RoleCard
            title="SLP"
            desc="Browse lessons, review assigned students, add notes, and check recognition accuracy and outcomes."
            to="/dashboard/slp"
          />
          <RoleCard
            title="Teacher"
            desc="Manage class lessons, track student completion, and leave session notes that sync with your workflow."
            to="/dashboard/teacher"
          />
          <RoleCard
            title="Student"
            desc="Open active lessons, see what is completed, and review guidance from your therapist or teacher."
            to="/dashboard/student"
          />
        </section>

        {/* Features */}
        <section className="mt-10 rounded-2xl bg-white p-6 shadow-sm ring-1 ring-gray-100">
          <h2 className="text-xl font-semibold">What you can do with Bloom</h2>
          <div className="mt-6 grid gap-6 sm:grid-cols-2 lg:grid-cols-3">
            <Feature
              title="Lesson management"
              text="Create and organize sessions that match classroom or clinic needs. Assign lessons to students and monitor progress in one place."
            />
            <Feature
              title="Notes and outcomes"
              text="Record observations per student and per lesson with a simple notes flow. Notes are saved now and can be synced to the backend later."
            />
            <Feature
              title="Recognition feedback"
              text="See accuracy trends for recognition with quick success and failure counts. Use this to adjust pacing and content."
            />
            <Feature
              title="Flexible deployment"
              text="Choose on-device or cloud components when you connect a robot. Monitor memory and latency to guide the decision."
            />
            <Feature
              title="Clean navigation"
              text="Move between roles and pages with consistent styling that matches the sign in and sign up experience."
            />
            <Feature
              title="Ready for integration"
              text="Current prototype uses local storage. The structure is ready to swap to your ASP.NET API for secure accounts and data."
            />
          </div>
        </section>

        {/* How it works */}
        <section className="mt-10 grid gap-6 lg:grid-cols-3">
          <Step
            number="1"
            title="Create an account"
            text="Pick a role during sign up. Bloom will route you to the correct dashboard and remember your session."
            cta={{ label: "Sign up", to: "/sign-up" }}
          />
          <Step
            number="2"
            title="Open your dashboard"
            text="Use the role dashboard to manage lessons, students, and notes. Admins can also watch system health."
            cta={{ label: session?.role ? "Open my dashboard" : "View dashboards", to: session?.role ? dashboardPathForRole(session.role) : "/sign-in" }}
          />
          <Step
            number="3"
            title="Connect the robot"
            text="When hardware is ready, hook up the telemetry and audio endpoints. The admin health cards will update from real data."
          />
        </section>

        {/* Footer links */}
        <section className="mt-12 flex flex-wrap items-center gap-3">
          <Link to="/fetch-data" className="rounded-md bg-white px-3 py-1.5 text-sm font-semibold text-indigo-700 ring-1 ring-inset ring-indigo-200 shadow-xs hover:bg-indigo-50">
            Sample data
          </Link>
          <Link to="/counter" className="rounded-md bg-white px-3 py-1.5 text-sm font-semibold text-indigo-700 ring-1 ring-inset ring-indigo-200 shadow-xs hover:bg-indigo-50">
            Counter demo
          </Link>
          {session?.role ? (
            <Link to={dashboardPathForRole(session.role)} className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500">
              Return to my dashboard
            </Link>
          ) : null}
        </section>
      </div>
    </div>
  );
}

/* ---------- helpers ---------- */

function RoleCard({ title, desc, to }) {
  return (
    <div className="rounded-2xl bg-white p-5 shadow-sm ring-1 ring-gray-100">
      <div className="text-sm font-semibold text-indigo-700">{title}</div>
      <p className="mt-2 text-sm text-gray-700">{desc}</p>
      <Link
        to={to}
        className="mt-4 inline-flex rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500"
      >
        Open
      </Link>
    </div>
  );
}

function Feature({ title, text }) {
  return (
    <div className="rounded-xl bg-gray-50 p-5 shadow-sm ring-1 ring-gray-100">
      <div className="text-base font-semibold">{title}</div>
      <p className="mt-2 text-sm text-gray-700">{text}</p>
    </div>
  );
}

function Step({ number, title, text, cta }) {
  return (
    <div className="rounded-2xl bg-white p-6 shadow-sm ring-1 ring-gray-100">
      <div className="inline-flex h-8 w-8 items-center justify-center rounded-full bg-indigo-600 text-sm font-semibold text-white">
        {number}
      </div>
      <h3 className="mt-3 text-base font-semibold">{title}</h3>
      <p className="mt-2 text-sm text-gray-700">{text}</p>
      {cta ? (
        <Link
          to={cta.to}
          className="mt-4 inline-flex rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white shadow-xs hover:bg-indigo-500"
        >
          {cta.label}
        </Link>
      ) : null}
    </div>
  );
}
