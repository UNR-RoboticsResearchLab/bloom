import { useEffect, useMemo, useState } from "react";
import DashboardLayout from "./DashboardLayout";

// Mock data; replace with API data when ready
const mockLessons = [
  { id: "L1", title: "R sound practice", students: ["S1", "S2"] },
  { id: "L2", title: "S blends", students: ["S1"] },
  { id: "L3", title: "Breath control", students: ["S2"] },
];

const mockStudents = {
  S1: { name: "Ava Martinez", completed: ["L2"], active: ["L1"] },
  S2: { name: "Liam Chen", completed: ["L3"], active: ["L1"] },
};

// Accuracy and outcomes by lesson and student
const mockSTT = {
  L1: { S1: { accuracy: 0.72, success: 12, fail: 4 }, S2: { accuracy: 0.86, success: 18, fail: 2 } },
  L2: { S1: { accuracy: 0.90, success: 24, fail: 3 } },
  L3: { S2: { accuracy: 0.81, success: 15, fail: 4 } },
};

// Simple notes store in localStorage
const NOTES_KEY = "lessonNotes"; // { [studentId]: { [lessonId]: [{text, ts}] } }

function useNotes() {
  const [map, setMap] = useState({});

  useEffect(() => {
    try {
      const raw = localStorage.getItem(NOTES_KEY);
      setMap(raw ? JSON.parse(raw) : {});
    } catch {
      setMap({});
    }
  }, []);

  function addNote(studentId, lessonId, text) {
    const next = { ...map };
    next[studentId] = next[studentId] || {};
    next[studentId][lessonId] = next[studentId][lessonId] || [];
    next[studentId][lessonId].push({ text, ts: Date.now() });
    setMap(next);
    localStorage.setItem(NOTES_KEY, JSON.stringify(next));
  }

  function getNotes(studentId, lessonId) {
    return map?.[studentId]?.[lessonId] || [];
  }

  return { addNote, getNotes };
}

function AccuracyBar({ value }) {
  const pct = Math.max(0, Math.min(1, value));
  return (
    <div className="w-full">
      <div className="h-2 w-full rounded bg-gray-200">
        <div className="h-2 rounded bg-indigo-600" style={{ width: `${pct * 100}%` }} />
      </div>
      <div className="mt-1 text-xs text-gray-600">{Math.round(pct * 100)}%</div>
    </div>
  );
}

export default function TeacherDashboard() {
  const [selectedLessonId, setSelectedLessonId] = useState("L1");
  const [selectedStudentId, setSelectedStudentId] = useState("S1");
  const { addNote, getNotes } = useNotes();

  const selectedLesson = useMemo(
    () => mockLessons.find((l) => l.id === selectedLessonId),
    [selectedLessonId]
  );

  const studentsForLesson = useMemo(
    () => (selectedLesson ? selectedLesson.students.map((sid) => ({ id: sid, ...mockStudents[sid] })) : []),
    [selectedLesson]
  );

  const sttForLesson = mockSTT[selectedLessonId] || {};
  // const selectedStudent = mockStudents[selectedStudentId];

  // Aggregate header stats
  const headerStats = useMemo(() => {
    const totalLessons = mockLessons.length;
    const totalStudents = new Set(mockLessons.flatMap((l) => l.students)).size;
    const accVals = Object.values(mockSTT).flatMap((obj) => Object.values(obj).map((v) => v.accuracy));
    const avgAcc = accVals.length ? accVals.reduce((a, b) => a + b, 0) / accVals.length : 0;
    return { totalLessons, totalStudents, avgAcc };
  }, []);

  function handleAddNote(e) {
    e.preventDefault();
    const form = e.currentTarget;
    const text = form.note.value.trim();
    if (!text) return;
    addNote(selectedStudentId, selectedLessonId, text);
    form.reset();
  }

  return (
    <DashboardLayout title="Teacher Dashboard">
      {/* Header stats */}
      <div className="grid gap-4 sm:grid-cols-3">
        <div className="rounded-lg bg-white p-4 shadow">
          <div className="text-sm text-gray-600">Lessons</div>
          <div className="mt-1 text-2xl font-semibold">{headerStats.totalLessons}</div>
        </div>
        <div className="rounded-lg bg-white p-4 shadow">
          <div className="text-sm text-gray-600">Students Assigned</div>
          <div className="mt-1 text-2xl font-semibold">{headerStats.totalStudents}</div>
        </div>
        <div className="rounded-lg bg-white p-4 shadow">
          <div className="text-sm text-gray-600">Avg STT Accuracy</div>
          <div className="mt-1 text-2xl font-semibold">{Math.round(headerStats.avgAcc * 100)}%</div>
        </div>
      </div>

      <div className="mt-6 grid gap-4 lg:grid-cols-3">
        {/* Lessons list */}
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Lessons</h3>
          <ul className="mt-3 divide-y">
            {mockLessons.map((lesson) => (
              <li key={lesson.id} className="py-2">
                <button
                  onClick={() => setSelectedLessonId(lesson.id)}
                  className={`w-full text-left ${selectedLessonId === lesson.id ? "font-semibold text-indigo-700" : "text-gray-800"} `}
                >
                  {lesson.title}
                </button>
                <div className="text-xs text-gray-500">
                  Students: {lesson.students.map((sid) => mockStudents[sid].name).join(", ")}
                </div>
              </li>
            ))}
          </ul>
        </section>

        {/* Students for selected lesson */}
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Students in Selected Lesson</h3>
          <ul className="mt-3 divide-y">
            {studentsForLesson.map((s) => (
              <li key={s.id} className="py-2">
                <button
                  onClick={() => setSelectedStudentId(s.id)}
                  className={`w-full text-left ${selectedStudentId === s.id ? "font-semibold text-indigo-700" : "text-gray-800"} `}
                >
                  {s.name}
                </button>
                <div className="mt-1 grid grid-cols-2 gap-3 text-xs text-gray-600">
                  <div>Active: {s.active.join(", ") || "None"}</div>
                  <div>Completed: {s.completed.join(", ") || "None"}</div>
                </div>
              </li>
            ))}
          </ul>
        </section>

        {/* STT accuracy and outcomes */}
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">STT Accuracy by Student</h3>
          <ul className="mt-3 space-y-3">
            {Object.entries(sttForLesson).map(([sid, m]) => (
              <li key={sid} className="rounded border p-3">
                <div className="flex items-center justify-between">
                  <div className="font-medium">{mockStudents[sid].name}</div>
                  <div className="text-sm text-gray-600">Success {m.success} • Fail {m.fail}</div>
                </div>
                <div className="mt-2">
                  <AccuracyBar value={m.accuracy} />
                </div>
              </li>
            ))}
            {!Object.keys(sttForLesson).length && (
              <li className="text-sm text-gray-600">No accuracy data yet</li>
            )}
          </ul>
        </section>
      </div>

      {/* Notes editor */}
      <div className="mt-6 grid gap-4 lg:grid-cols-2">
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Add Note</h3>
          <p className="mt-1 text-sm text-gray-600">
            Notes are stored in your browser for now and can be moved to your API later.
          </p>
          <form onSubmit={handleAddNote} className="mt-3 space-y-3">
            <div className="grid gap-3 sm:grid-cols-2">
              <div>
                <label className="text-xs text-gray-600">Lesson</label>
                <select
                  value={selectedLessonId}
                  onChange={(e) => setSelectedLessonId(e.target.value)}
                  className="mt-1 block w-full rounded-md border-gray-300 text-sm"
                >
                  {mockLessons.map((l) => (
                    <option key={l.id} value={l.id}>{l.title}</option>
                  ))}
                </select>
              </div>
              <div>
                <label className="text-xs text-gray-600">Student</label>
                <select
                  value={selectedStudentId}
                  onChange={(e) => setSelectedStudentId(e.target.value)}
                  className="mt-1 block w-full rounded-md border-gray-300 text-sm"
                >
                  {Object.entries(mockStudents).map(([sid, s]) => (
                    <option key={sid} value={sid}>{s.name}</option>
                  ))}
                </select>
              </div>
            </div>
            <textarea
              name="note"
              rows={3}
              placeholder="Write a brief note…"
              className="block w-full rounded-md border-gray-300 p-2 text-sm"
            />
            <button
              type="submit"
              className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500"
            >
              Save Note
            </button>
          </form>
        </section>

        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Notes for Selection</h3>
          <ul className="mt-3 space-y-2 text-sm">
            {getNotes(selectedStudentId, selectedLessonId).length === 0 && (
              <li className="text-gray-600">No notes yet</li>
            )}
            {getNotes(selectedStudentId, selectedLessonId).map((n, i) => (
              <li key={i} className="rounded border p-2">
                <div className="text-gray-800">{n.text}</div>
                <div className="mt-1 text-xs text-gray-500">
                  {new Date(n.ts).toLocaleString()}
                </div>
              </li>
            ))}
          </ul>
        </section>
      </div>
    </DashboardLayout>
  );
}
