import { useMemo, useState, useEffect} from "react";
import DashboardLayout from "./DashboardLayout";
import LessonBuilder from "../LessonBuilder";
import { useApiClient } from "../../context/ApiClientContext";

// function Stat({ label, value, unit = "", percent = null }) {
//   return (
//     <div className="rounded-lg bg-white p-4 shadow">
//       <div className="text-sm text-gray-600">{label}</div>
//       <div className="mt-1 text-2xl font-semibold">
//         {value}{unit && <span className="text-sm font-normal text-gray-600 ml-1">{unit}</span>}
//       </div>
//       {typeof percent === "number" && (
//         <div className="mt-3">
//           <div className="h-2 w-full rounded bg-gray-200">
//             <div
//               className="h-2 rounded bg-indigo-600"
//               style={{ width: `${Math.max(0, Math.min(100, percent))}%` }}
//             />
//           </div>
//           <div className="mt-1 text-xs text-gray-500">{percent.toFixed(0)}%</div>
//         </div>
//       )}
//     </div>
//   );
// }

function SeverityTag({ severity }) {
  const styles = {
    info: "bg-blue-100 text-blue-700",
    warning: "bg-yellow-100 text-yellow-700",
    error: "bg-red-100 text-red-700",
  };

  return (
    <span className={`inline-flex rounded-full px-2.5 py-1 text-xs font-semibold ${styles[severity] || styles.info}`}>
      {(severity || "info").toUpperCase()}    </span>
  );
}

export default function AdminDashboard() {
  const api = useApiClient();
  // const api = null;
  const [lessonPaneOpen, setLessonPaneOpen] = useState(false);
  const [lessonSuccess, setLessonSuccess] = useState("");
  const [events, setEvents] = useState([]);
  const [sessions, setSessions] = useState([]);
  const [selectedSessionId, setSelectedSessionId] = useState("");
  // Simulated readings; replace with real values from your API later
  // const [tick, setTick] = useState(0);

  // const readings = useMemo(() => {
  //   const rand = (min, max) => Math.random() * (max - min) + min;

  //   const memUsedPct = rand(28, 76);
  //   const speakerLatency = rand(18, 45);   // ms
  //   const micLatency = rand(10, 30);       // ms
  //   const networkLatency = rand(25, 90);   // ms
  //   const sttError = rand(2, 9);           // %
  //   const ttsError = rand(1, 6);           // %

  //   return {
  //     memUsedPct,
  //     speakerLatency,
  //     micLatency,
  //     networkLatency,
  //     sttError,
  //     ttsError,
  //     updatedAt: new Date().toLocaleTimeString(),
  //   };
  // }, [tick]);

  // function refresh() {
  //   setTick((t) => t + 1);
  // }

  useEffect(() => {
    async function loadSessions() {
      try {
        const data = await api.getSessions(); // make sure this exists
        console.log("sessions:", data);

        setSessions(data);

        // auto select first session
        if (data.length > 0) {
          setSelectedSessionId(data[0].id);
        }
      } catch (err) {
        console.error("failed to load sessions:", err);
      }
    }

    loadSessions();
  }, [api]);

  // Load events from API on mount
  useEffect(() => {
    if (!selectedSessionId || selectedSessionId === "all") return;

    let mounted = true;

    async function loadEvents() {
      try {
        const data = await api.getTrackerEvents(selectedSessionId);

        if (mounted) {
          console.log("tracker events:", data);
          setEvents(data);
        }
      } catch (err) {
        console.error("tracker fetch failed:", err);
      }
    }

    loadEvents();

    return () => {
      mounted = false;
    };
  }, [api, selectedSessionId]);

  //Mock event data
  // const events = useMemo(() => {
  //   return [
  //     {
  //       id: 1,
  //       timestamp: "10:42:11 AM",
  //       severity: "info",
  //       source: "Session",
  //       eventType: "session_started",
  //       message: "Robot session started successfully",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 2,
  //       timestamp: "10:42:34 AM",
  //       severity: "info",
  //       source: "Robot",
  //       eventType: "robot_joined",
  //       message: "Robot BLOSSOM 01 joined the active session",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 3,
  //       timestamp: "10:43:02 AM",
  //       severity: "info",
  //       source: "Lesson",
  //       eventType: "lesson_attached",
  //       message: "Lesson was attached to the session",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 4,
  //       timestamp: "10:43:18 AM",
  //       severity: "warning",
  //       source: "Speech To Text",
  //       eventType: "stt_latency_high",
  //       message: "Speech recognition latency exceeded expected threshold",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 5,
  //       timestamp: "10:43:41 AM",
  //       severity: "error",
  //       source: "Speech To Text",
  //       eventType: "stt_timeout",
  //       message: "Speech recognition request timed out after 5 seconds",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 6,
  //       timestamp: "10:44:05 AM",
  //       severity: "warning",
  //       source: "Audio",
  //       eventType: "microphone_reconnected",
  //       message: "Microphone input was lost and then reconnected",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 7,
  //       timestamp: "10:44:22 AM",
  //       severity: "info",
  //       source: "Text To Speech",
  //       eventType: "tts_completed",
  //       message: "Robot speech output completed successfully",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 8,
  //       timestamp: "10:44:49 AM",
  //       severity: "warning",
  //       source: "Network",
  //       eventType: "network_latency_spike",
  //       message: "Network latency spike detected during lesson playback",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 9,
  //       timestamp: "10:45:10 AM",
  //       severity: "info",
  //       source: "Notes",
  //       eventType: "note_added",
  //       message: "SLP note added to session timeline",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //     {
  //       id: 10,
  //       timestamp: "10:45:37 AM",
  //       severity: "info",
  //       source: "Session",
  //       eventType: "session_ended",
  //       message: "Robot session ended normally",
  //       sessionId: "RS 2041",
  //       lessonTitle: "Animal Sounds Practice",
  //       studentName: "Alex Carter",
  //     },
  //   ];
  // }, []);

  // filter state for events log. 
  const [filter, setFilter] = useState("all");
  const [systemFilter, setSystemFilter] = useState("all");
  const [studentFilter, setStudentFilter] = useState("all");
  const [lessonFilter, setLessonFilter] = useState("all");
  // const [sessionFilter, setSessionFilter] = useState("all");

  const filteredEvents = useMemo(() => {
    return events.filter((e) => {
      const severityMatch =
        filter === "all" || (e.severity || e.Severity) === filter;

      const systemMatch =
        systemFilter === "all" || (e.source || e.Source) === systemFilter;

      const studentMatch =
        studentFilter === "all" || (e.studentName || e.StudentName) === studentFilter;

      const lessonMatch =
        lessonFilter === "all" || (e.lessonTitle || e.LessonTitle) === lessonFilter;

      // const sessionMatch =
      //   sessionFilter === "all" || (e.sessionId || e.SessionId) === sessionFilter;

      return (
        severityMatch &&
        systemMatch &&
        studentMatch &&
        lessonMatch
        //  &&
        // sessionMatch
      );
    });
  }, [events, filter, systemFilter, studentFilter, lessonFilter]);
  // , sessionFilter
  const studentOptions = useMemo(() => {
    return ["all", ...new Set(events.map(e => e.studentName || e.StudentName).filter(Boolean))];
  }, [events]);

  const lessonOptions = useMemo(() => {
    return ["all", ...new Set(events.map(e => e.lessonTitle || e.LessonTitle).filter(Boolean))];
  }, [events]);

  // const sessionOptions = useMemo(() => {
  //   return ["all", ...sessions.map(s => s.id)];
  // }, [sessions]);

  const sessionOptions = sessions.map(s => s.id);

  const systemOptions = useMemo(() => {
    return ["all", ...new Set(events.map(e => e.source || e.Source).filter(Boolean))];
  }, [events]);

  // async function handleLessonSubmit(dto) {
  //   await api.createLesson(dto);
  //   setLessonSuccess(`"${dto.title}" saved.`);
  //   setLessonPaneOpen(false);
  // }

  return (
    <DashboardLayout title="Admin Dashboard">
      {/* <div className="mb-4 flex items-center justify-between">
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
      </div> */}

      {/* Placeholder for system events log; replace with real event data from your API */}
      <section className="mt-6 rounded-lg bg-white p-4 border border-gray-200">
        <div className="mb-4 ">
          <h3 className="text-base font-semibold">System Events</h3>
          <p className="mt-1 text-sm text-gray-600">
            Recent activity, warnings, and errors from Bloom sessions.
          </p>
          <div className="mb-4 flex flex-wrap items-start gap-3 ">
            {/* Severity Filter */}
            <div className="flex flex-col gap-1 rounded-lg border border-gray-200 py-4 px-3 bg-gray-400/10">
              <label
                htmlFor="severityFilter"
                className="text-lg font-medium text-center"
              >
                Severity
              </label>

              <select
                id="severityFilter"
                value={filter}
                onChange={(e) => setFilter(e.target.value)}
                className="min-w-[160px] max-w-[160px] rounded-md border border-gray-300 bg-gray-400/50 px-3 py-2 text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              >
                <option value="all">All</option>
                <option value="info">Info</option>
                <option value="warning">Warning</option>
                <option value="error">Error</option>
              </select>
            </div>
            {/* System Filter */}
            <div className="flex flex-col gap-1 rounded-lg border border-gray-200 py-4 px-3 bg-gray-400/10">
              <label className="text-lg font-medium text-center">
                System
              </label>

              <select
                value={systemFilter}
                onChange={(e) => setSystemFilter(e.target.value)}
                className="min-w-[160px] max-w-[160px] rounded-md border border-gray-300 bg-gray-400/50 px-3 py-2 text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              >
                {systemOptions.map((opt) => (
                  <option key={`system-${opt}`} value={opt}>
                    {opt === "all" ? "All" : opt}
                  </option>
                ))}
              </select>
            </div>
            {/* Student Filter */}
            <div className="flex flex-col gap-1 rounded-lg border border-gray-200 py-4 px-3 bg-gray-400/10">
              <label className="text-lg font-medium text-center">
                Student
              </label>

              <select
                value={studentFilter}
                onChange={(e) => setStudentFilter(e.target.value)}
                 className="min-w-[160px] max-w-[160px] rounded-md border border-gray-300 bg-gray-400/50 px-3 py-2 text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              >
                {studentOptions.map((opt) => (
                  <option key={`student-${opt}`} value={opt}>
                    {opt === "all" ? "All" : opt}
                  </option>
                ))}
              </select>
            </div>
            {/* Lesson Filter */}
            <div className="flex flex-col gap-1 rounded-lg border border-gray-200 py-4 px-3 bg-gray-400/10">
              <label className="text-lg font-medium text-center">
                Lesson
              </label>

              <select
                value={lessonFilter}
                onChange={(e) => setLessonFilter(e.target.value)}
                className="min-w-[160px] max-w-[160px] rounded-md border border-gray-300 bg-gray-400/50 px-3 py-2 text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              >
                {lessonOptions.map((opt) => (
                  <option key={`lesson-${opt}`} value={opt}>
                    {opt === "all" ? "All" : opt}
                  </option>
                ))}
              </select>
            </div>
            {/* Session Filter */}
            <div className="flex flex-col gap-1 rounded-lg border border-gray-200 py-4 px-3 bg-gray-400/10">
              <label className="text-lg font-medium text-center">
                Session
              </label>

              <select
                value={selectedSessionId}
                onChange={(e) => setSelectedSessionId(e.target.value)}
                 className="min-w-[160px] max-w-[160px] rounded-md border border-gray-300 bg-gray-400/50 px-3 py-2 text-sm text-gray-700 shadow-sm focus:border-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-200"
              >
                {sessionOptions.map((opt) => (
                  <option key={`session-${opt}`} value={opt}>
                    {opt === "all" ? "All" : opt}
                  </option>
                ))}
              </select>
            </div>
          </div>
        </div>

        <div className="overflow-hidden rounded-lg border border-gray-200">
          <div className="max-h-[1000px] overflow-y-auto">
            {filteredEvents.length === 0 ? (
                <p className="p-4 text-sm text-gray-500">No events found.</p>
              ) : (
              filteredEvents.map((event, index) => (
              <div
                key={event.id || event.Id || index}
                className="border-b border-gray-100 px-4 py-4 last:border-b-0"
              >
                <div className="flex flex-col gap-3 md:flex-row md:items-start md:justify-between">
                  <div className="space-y-2">
                    <div className="flex flex-wrap items-center gap-2">
                      <SeverityTag severity={event.severity || event.Severity} />
                      <span className="text-sm font-medium text-gray-800">{event.source || event.Source}</span>
                      <span className="text-sm text-gray-500">
                        {/* {event.timestamp} */}
                        {new Date(event.timestamp || event.Timestamp).toLocaleTimeString([], {
                          hour: "2-digit",
                          minute: "2-digit",
                        })}
                      </span>
                    </div>

                    <p className="text-sm text-gray-800">{event.message || event.Message}</p>

                    <div className="flex flex-wrap gap-2 text-xs text-gray-500">
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Session: {event.sessionId || event.SessionId}
                      </span>
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Lesson: {event.lessonTitle || event.LessonTitle}
                      </span>
                      <span className="rounded-full bg-gray-100 px-2 py-1">
                        Student: {event.studentName || event.StudentName}
                      </span>
                    </div>
                  </div>
                </div>
              </div>
            )))}
          </div>
        </div>
      </section>

      <div className="mt-6">
        <section className="rounded-lg bg-white p-4 border border-gray-200">
          <h3 className="text-base font-semibold">Lesson Management</h3>
          <p className="mt-2 text-sm text-gray-600">Create and manage lessons for the system.</p>
          <button
            onClick={() => setLessonPaneOpen(true)}
            className="mt-4 w-full rounded-md bg-green-600 px-4 py-2 text-sm font-semibold text-white hover:bg-green-500"
          >
            Add Lesson
          </button>
        </section>

      </div>
      {lessonPaneOpen && (
        <div className="fixed inset-0 z-40 flex items-center justify-center p-4">
          <div
            className="absolute inset-0 bg-black/30"
            onClick={() => setLessonPaneOpen(false)}
          />
          <div className="relative z-50 w-full max-w-2xl max-h-[90vh] overflow-y-auto bg-gray-50 shadow-xl rounded-xl">
            <div className="flex items-center justify-between border-b border-gray-200 bg-white px-6 py-4">
              <h2 className="text-lg font-semibold text-gray-900">New Lesson</h2>
              <button
                onClick={() => setLessonPaneOpen(false)}
                className="text-gray-400 hover:text-gray-600 text-xl leading-none"
              >
                &times;
              </button>
            </div>
            {/* <LessonBuilder
              onSubmit={handleLessonSubmit}
              onCancel={() => setLessonPaneOpen(false)}
            /> */}
          </div>
        </div>
      )}

      {lessonSuccess && (
        <div className="fixed bottom-4 right-4 z-50 rounded-md bg-green-600 px-4 py-2 text-sm font-medium text-white shadow">
          {lessonSuccess}
          <button
            onClick={() => setLessonSuccess("")}
            className="ml-3 text-green-100 hover:text-white"
          >
            &times;
          </button>
        </div>
      )}
    </DashboardLayout>
  );
}