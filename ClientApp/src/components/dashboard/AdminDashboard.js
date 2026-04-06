import { useMemo, useState } from "react";
import DashboardLayout from "./DashboardLayout";
import LessonBuilder from "../LessonBuilder";
import { useApiClient } from "../../context/ApiClientContext";

function Stat({ label, value, unit = "", percent = null }) {
  return (
    <div className="rounded-lg bg-white p-4 shadow">
      <div className="text-sm text-gray-600">{label}</div>
      <div className="mt-1 text-2xl font-semibold">
        {value}{unit && <span className="text-sm font-normal text-gray-600 ml-1">{unit}</span>}
      </div>
      {typeof percent === "number" && (
        <div className="mt-3">
          <div className="h-2 w-full rounded bg-gray-200">
            <div
              className="h-2 rounded bg-indigo-600"
              style={{ width: `${Math.max(0, Math.min(100, percent))}%` }}
            />
          </div>
          <div className="mt-1 text-xs text-gray-500">{percent.toFixed(0)}%</div>
        </div>
      )}
    </div>
  );
}

export default function AdminDashboard() {
  const api = useApiClient();
  const [lessonPaneOpen, setLessonPaneOpen] = useState(false);
  const [lessonSuccess, setLessonSuccess] = useState("");

  async function handleLessonSubmit(dto) {
    await api.createLesson(dto);
    setLessonSuccess(`"${dto.title}" saved.`);
    setLessonPaneOpen(false);
  }

  // Simulated readings; replace with real values from your API later
  const [tick, setTick] = useState(0);
  const readings = useMemo(() => {
    const rand = (min, max) => Math.random() * (max - min) + min;

    const memUsedPct = rand(28, 76);
    const speakerLatency = rand(18, 45);   // ms
    const micLatency = rand(10, 30);       // ms
    const networkLatency = rand(25, 90);   // ms
    const sttError = rand(2, 9);           // %
    const ttsError = rand(1, 6);           // %

    return {
      memUsedPct,
      speakerLatency,
      micLatency,
      networkLatency,
      sttError,
      ttsError,
      updatedAt: new Date().toLocaleTimeString(),
    };
  }, [tick]);

  function refresh() {
    setTick((t) => t + 1);
  }

  return (
    <DashboardLayout title="Admin Dashboard">
      <div className="mb-4 flex items-center justify-between">
        <div className="text-sm text-gray-600">Last updated: {readings.updatedAt}</div>
        <div className="flex items-center gap-2">
          <button
            onClick={() => setLessonPaneOpen(true)}
            className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500"
          >
            + Add Lesson
          </button>
          <button
            onClick={refresh}
            className="rounded-md border border-gray-300 px-3 py-1.5 text-sm font-semibold text-gray-700 hover:bg-gray-50"
          >
            Refresh
          </button>
        </div>
      </div>

      {lessonSuccess && (
        <div className="mb-4 rounded-md bg-green-50 px-4 py-2 text-sm text-green-700">
          {lessonSuccess}
        </div>
      )}

      {/* Add Lesson modal */}
      {lessonPaneOpen && (
        <div className="fixed inset-0 z-40 flex items-center justify-center p-4">
          <div
            className="absolute inset-0 bg-black/40"
            onClick={() => setLessonPaneOpen(false)}
          />
          <div className="relative z-50 w-full max-w-2xl max-h-[90vh] overflow-y-auto rounded-xl bg-gray-50 shadow-xl">
            <div className="flex items-center justify-between border-b border-gray-200 bg-white px-6 py-4 rounded-t-xl">
              <h2 className="text-base font-semibold text-gray-900">Add Lesson</h2>
              <button
                onClick={() => setLessonPaneOpen(false)}
                className="text-gray-400 hover:text-gray-600"
              >
                ✕
              </button>
            </div>
            <LessonBuilder
              onSubmit={handleLessonSubmit}
              onCancel={() => setLessonPaneOpen(false)}
            />
          </div>
        </div>
      )}

      <div className="grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
        <Stat label="PI Memory Usage" value={readings.memUsedPct.toFixed(0)} unit="%" percent={readings.memUsedPct} />
        <Stat label="Speaker Latency" value={readings.speakerLatency.toFixed(0)} unit="ms" />
        <Stat label="Microphone Latency" value={readings.micLatency.toFixed(0)} unit="ms" />
        <Stat label="Network Latency" value={readings.networkLatency.toFixed(0)} unit="ms" />
        <Stat label="STT Error Rate" value={readings.sttError.toFixed(1)} unit="%" percent={readings.sttError} />
        <Stat label="TTS Error Rate" value={readings.ttsError.toFixed(1)} unit="%" percent={readings.ttsError} />
      </div>

      <div className="mt-6 grid gap-4 lg:grid-cols-2">
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Health Notes</h3>
          <ul className="mt-3 list-disc pl-5 text-sm text-gray-700">
            <li>Use this page to compare cloud vs on-device placement for STT, LLM, and TTS once the robot is live.</li>
            <li>Replace the random values with metrics from your telemetry endpoint.</li>
          </ul>
        </section>

        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Planned Metrics</h3>
          <ul className="mt-3 list-disc pl-5 text-sm text-gray-700">
            <li>CPU temperature and throttling state</li>
            <li>Audio device availability and reconnect count</li>
            <li>Robust network quality score over time</li>
          </ul>
        </section>
      </div>
    </DashboardLayout>
  );
}
