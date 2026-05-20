import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";

const STEPS = ["pair", "lesson", "name"];

export default function Demo() {
  const navigate = useNavigate();
  const api = useApiClient();

  const [step, setStep] = useState(STEPS[0]);
  const [robotCode, setRobotCode] = useState("");
  const [sessionId, setSessionId] = useState(localStorage.getItem("pairedSessionId") || "");
  const [lessons, setLessons] = useState([]);
  const [selectedLesson, setSelectedLesson] = useState(null);
  const [studentName, setStudentName] = useState("");
  const [error, setError] = useState("");
  const [isLoading, setIsLoading] = useState(false);

  // Skip pairing step if already paired
  useEffect(() => {
    if (sessionId) setStep(STEPS[1]);
  }, []);

  useEffect(() => {
    if (step !== "lesson") return;
    api.getLessons()
      .then((data) => setLessons(Array.isArray(data) ? data : []))
      .catch(() => setLessons([]));
  }, [step, api]);

  async function handlePair(e) {
    e.preventDefault();
    if (!robotCode.trim()) return;
    setError("");
    setIsLoading(true);
    try {
      const res = await api.getSessionIdFromRobotCode(robotCode.trim());
      const id = res?.id ?? res?.Id;
      if (!id) throw new Error("No session ID returned.");
      localStorage.setItem("pairedSessionId", id);
      localStorage.setItem("pairedRobotCode", robotCode.trim());
      setSessionId(id);
      setStep(STEPS[1]);
    } catch {
      setError("Could not find a session for that code. Check the robot and try again.");
    } finally {
      setIsLoading(false);
    }
  }

  function handleLessonSelect(lesson) {
    setSelectedLesson(lesson);
    setStep(STEPS[2]);
  }

  function handleStart(e) {
    e.preventDefault();
    const name = studentName.trim() || "Demo User";
    navigate("/lesson-view", {
      state: {
        lesson: selectedLesson,
        student: { name, fullName: name, id: sessionId },
      },
    });
  }

  function handleUnpair() {
    localStorage.removeItem("pairedSessionId");
    localStorage.removeItem("pairedRobotCode");
    setSessionId("");
    setStep(STEPS[0]);
    setSelectedLesson(null);
  }

  const stepIndex = STEPS.indexOf(step);

  return (
    <div className="mx-auto mt-10 max-w-xl space-y-6 px-4">
      <div>
        <p className="text-xs font-semibold uppercase tracking-wide text-indigo-600">
          Demo Mode
        </p>
        <h1 className="mt-1 text-2xl font-bold text-gray-900">Launch a Lesson</h1>
        <p className="mt-1 text-sm text-gray-500">
          No account required - pair a robot, pick a lesson, and go.
        </p>
      </div>

      {/* Progress indicator */}
      <div className="flex items-center gap-2">
        {["Pair Robot", "Select Lesson", "Start"].map((label, i) => (
          <div key={label} className="flex items-center gap-2">
            <div
              className={`flex h-7 w-7 items-center justify-center rounded-full text-xs font-bold
                ${i < stepIndex ? "bg-indigo-600 text-white"
                  : i === stepIndex ? "bg-indigo-100 text-indigo-700 ring-2 ring-indigo-500"
                  : "bg-gray-100 text-gray-400"}`}
            >
              {i < stepIndex ? "✓" : i + 1}
            </div>
            <span className={`text-sm ${i === stepIndex ? "font-semibold text-gray-900" : "text-gray-400"}`}>
              {label}
            </span>
            {i < 2 && <span className="text-gray-300">›</span>}
          </div>
        ))}
      </div>

      {/* Step 1: Pair */}
      {step === "pair" && (
        <div className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
          <h2 className="text-base font-semibold text-gray-900">Enter Robot Code</h2>
          <p className="mt-1 text-sm text-gray-500">
            Find the 6-digit code displayed on the robot.
          </p>
          <form onSubmit={handlePair} className="mt-4 space-y-4">
            <input
              value={robotCode}
              onChange={(e) => setRobotCode(e.target.value)}
              placeholder="e.g. 123456"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              autoFocus
            />
            {error && (
              <p className="rounded-md border border-red-200 bg-red-50 px-3 py-2 text-sm text-red-700">
                {error}
              </p>
            )}
            <button
              type="submit"
              disabled={isLoading}
              className="w-full rounded-lg bg-indigo-600 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500 disabled:opacity-60"
            >
              {isLoading ? "Connecting..." : "Pair Robot"}
            </button>
          </form>
        </div>
      )}

      {/* Step 2: Select Lesson */}
      {step === "lesson" && (
        <div className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
          <div className="flex items-center justify-between">
            <div>
              <h2 className="text-base font-semibold text-gray-900">Select a Lesson</h2>
              <p className="mt-1 text-sm text-gray-500">
                Robot paired
                {localStorage.getItem("pairedRobotCode")
                  ? ` · Code: ${localStorage.getItem("pairedRobotCode")}`
                  : ""}
              </p>
            </div>
            <button
              onClick={handleUnpair}
              className="text-xs text-gray-400 underline hover:text-gray-600"
            >
              Change robot
            </button>
          </div>

          <div className="mt-4 max-h-96 space-y-2 overflow-y-auto">
            {lessons.length === 0 ? (
              <p className="text-sm text-gray-500">Loading lessons…</p>
            ) : (
              lessons.map((lesson) => {
                const id = lesson.id ?? lesson.Id;
                const title = lesson.title ?? lesson.Title ?? "Untitled";
                const description = lesson.description ?? lesson.Description ?? "";
                return (
                  <button
                    key={id}
                    onClick={() => handleLessonSelect(lesson)}
                    className="w-full rounded-lg border border-gray-200 bg-gray-50 px-4 py-3 text-left shadow-sm transition hover:border-indigo-300 hover:bg-indigo-50"
                  >
                    <p className="text-sm font-semibold text-gray-900">{title}</p>
                    {description && (
                      <p className="mt-0.5 line-clamp-2 text-xs text-gray-500">{description}</p>
                    )}
                  </button>
                );
              })
            )}
          </div>
        </div>
      )}

      {/* Step 3: Student name + launch */}
      {step === "name" && (
        <div className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
          <h2 className="text-base font-semibold text-gray-900">Who is this session for?</h2>
          <p className="mt-1 text-sm text-gray-500">
            A name for the transcript. Leave blank to use "Demo User".
          </p>

          <div className="mt-3 rounded-lg border border-indigo-100 bg-indigo-50 px-4 py-3 text-sm text-indigo-800">
            <span className="font-semibold">Lesson:</span>{" "}
            {selectedLesson?.title ?? selectedLesson?.Title ?? "Untitled"}
          </div>

          <form onSubmit={handleStart} className="mt-4 space-y-4">
            <input
              value={studentName}
              onChange={(e) => setStudentName(e.target.value)}
              placeholder="Student name (optional)"
              className="w-full rounded-lg border border-gray-300 px-3 py-2 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
              autoFocus
            />
            <div className="flex gap-3">
              <button
                type="button"
                onClick={() => setStep(STEPS[1])}
                className="flex-1 rounded-lg border border-gray-300 py-2 text-sm font-medium text-gray-700 hover:bg-gray-50"
              >
                Back
              </button>
              <button
                type="submit"
                className="flex-1 rounded-lg bg-indigo-600 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500"
              >
                Start Lesson
              </button>
            </div>
          </form>
        </div>
      )}
    </div>
  );
}
