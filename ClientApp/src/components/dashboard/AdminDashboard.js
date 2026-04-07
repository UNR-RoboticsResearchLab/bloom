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

function SeverityTag({ severity }) {
  const styles = {
    info: "bg-blue-100 text-blue-700",
    warning: "bg-yellow-100 text-yellow-700",
    error: "bg-red-100 text-red-700",
  };

  return (
    <span className={`inline-flex rounded-full px-2.5 py-1 text-xs font-semibold ${styles[severity] || styles.info}`}>
      {severity.toUpperCase()}
    </span>
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

  //Mock event data
  const events = useMemo(() => {
    return [
      {
        id: 1,
        timestamp: "10:42:11 AM",
        severity: "info",
        source: "Session",
        eventType: "session_started",
        message: "Robot session started successfully",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 2,
        timestamp: "10:42:34 AM",
        severity: "info",
        source: "Robot",
        eventType: "robot_joined",
        message: "Robot BLOSSOM 01 joined the active session",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 3,
        timestamp: "10:43:02 AM",
        severity: "info",
        source: "Lesson",
        eventType: "lesson_attached",
        message: "Lesson was attached to the session",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 4,
        timestamp: "10:43:18 AM",
        severity: "warning",
        source: "Speech To Text",
        eventType: "stt_latency_high",
        message: "Speech recognition latency exceeded expected threshold",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 5,
        timestamp: "10:43:41 AM",
        severity: "error",
        source: "Speech To Text",
        eventType: "stt_timeout",
        message: "Speech recognition request timed out after 5 seconds",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 6,
        timestamp: "10:44:05 AM",
        severity: "warning",
        source: "Audio",
        eventType: "microphone_reconnected",
        message: "Microphone input was lost and then reconnected",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 7,
        timestamp: "10:44:22 AM",
        severity: "info",
        source: "Text To Speech",
        eventType: "tts_completed",
        message: "Robot speech output completed successfully",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 8,
        timestamp: "10:44:49 AM",
        severity: "warning",
        source: "Network",
        eventType: "network_latency_spike",
        message: "Network latency spike detected during lesson playback",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 9,
        timestamp: "10:45:10 AM",
        severity: "info",
        source: "Notes",
        eventType: "note_added",
        message: "SLP note added to session timeline",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
      {
        id: 10,
        timestamp: "10:45:37 AM",
        severity: "info",
        source: "Session",
        eventType: "session_ended",
        message: "Robot session ended normally",
        sessionId: "RS 2041",
        lessonTitle: "Animal Sounds Practice",
        studentName: "Alex Carter",
      },
    ];
  }, []);

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

      {/* Placeholder for system events log; replace with real event data from your API */}
      <section className="mt-6 rounded-lg bg-white p-4 shadow">
        <div className="mb-4">
          <h3 className="text-base font-semibold">System Events</h3>
          <p className="mt-1 text-sm text-gray-600">
            Recent activity, warnings, and errors from Bloom sessions.
          </p>
        </div>

        <div className="overflow-hidden rounded-lg border border-gray-200">
          <div className="max-h-[420px] overflow-y-auto">
            {events.map((event) => (
              <div
                key={event.id}
                className="border-b border-gray-100 px-4 py-4 last:border-b-0"
              >
                <div className="flex flex-col gap-3 md:flex-row md:items-start md:justify-between">
                  <div className="space-y-2">
                    <div className="flex flex-wrap items-center gap-2">
                      <SeverityTag severity={event.severity} />
                      <span className="text-sm font-medium text-gray-800">{event.source}</span>
                      <span className="text-sm text-gray-500">{event.timestamp}</span>
                    </div>

                    <p className="text-sm text-gray-800">{event.message}</p>

                    <div className="flex flex-wrap gap-2 text-xs text-gray-500">
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Session: {event.sessionId}
                      </span>
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Lesson: {event.lessonTitle}
                      </span>
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Student: {event.studentName}
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            ))}
          </div>
        </div>
      </section>


      <div className="mt-6 grid gap-4 sm:grid-cols-2 lg:grid-cols-3">
        <Stat label="PI Memory Usage" value={readings.memUsedPct.toFixed(0)} unit="%" percent={readings.memUsedPct} />
        <Stat label="Speaker Latency" value={readings.speakerLatency.toFixed(0)} unit="ms" />
        <Stat label="Microphone Latency" value={readings.micLatency.toFixed(0)} unit="ms" />
        <Stat label="Network Latency" value={readings.networkLatency.toFixed(0)} unit="ms" />
        <Stat label="STT Error Rate" value={readings.sttError.toFixed(1)} unit="%" percent={readings.sttError} />
        <Stat label="TTS Error Rate" value={readings.ttsError.toFixed(1)} unit="%" percent={readings.ttsError} />
      </div>

      <div className="mt-6 grid gap-4 lg:grid-cols-3">
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Lesson Management</h3>
          <p className="mt-2 text-sm text-gray-600">Create and manage lessons for the system.</p>
          <button
            onClick={() => window.location.href = "/dashboard/admin/add-lesson"}
            className="mt-4 w-full rounded-md bg-green-600 px-4 py-2 text-sm font-semibold text-white hover:bg-green-500"
          >
            Add Lesson
          </button>
        </section>

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
