import { useEffect, useMemo, useState } from "react";
import { useNavigate } from "react-router-dom";
import DashboardLayout from "./DashboardLayout";
import { LessonCard } from "../../pages/LessonCard";
import { StudentCard } from "../../pages/StudentCard";
import { PairRobotCard } from "../../pages/PairRobotCard";
import { useApiClient } from "../../context/ApiClientContext";
import { useRobotPairing } from "../../context/RobotPairingContext";

// Same mock data as Teacher dashboard
const mockLessons = [
  { id: "L1", title: "R sound practice", students: ["S1", "S2"] },
  { id: "L2", title: "S blends", students: ["S1"] },
  { id: "L3", title: "Breath control", students: ["S2"] },
];

const mockStudents = {
  S1: { name: "Ava Martinez", completed: ["L2"], active: ["L1"] },
  S2: { name: "Liam Chen", completed: ["L3"], active: ["L1"] },
};

const mockSTT = {
  L1: { S1: { accuracy: 0.72, success: 12, fail: 4 }, S2: { accuracy: 0.86, success: 18, fail: 2 } },
  L2: { S1: { accuracy: 0.90, success: 24, fail: 3 } },
  L3: { S2: { accuracy: 0.81, success: 15, fail: 4 } },
};

const NOTES_KEY = "lessonNotes";

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

export default function SlpDashboard() {
  const navigate = useNavigate();
  const apiClient = useApiClient();
  const [selectedLessonId, setSelectedLessonId] = useState("");
  const [selectedStudentId, setSelectedStudentId] = useState("");
  const { addNote, getNotes } = useNotes();
  const [showPairRobotCard, setShowPairRobotCard] = useState(false);
  const { isPaired } = useRobotPairing();
  const [lessons, setLessons] = useState([]);

  const [students, setStudents] = useState([]);

  useEffect(() => {
    async function loadLessons() {
      try {
        const data = await apiClient.getLessons();
        setLessons(Array.isArray(data) ? data : []);
      } catch (error) {
        console.error("Failed to load lessons:", error);
        setLessons([]);
      }
    }
    loadLessons();
  }, [apiClient]);

  useEffect(() => {
    async function loadStudents() {
      try {
        const data = await apiClient.getStudents();

        const normalizedStudents = Array.isArray(data)
          ? data.map((client) => ({
              id: client.studentId,
              fullName: client.studentName,
              email: client.email ?? "N/A",
            }))
          : [];

        setStudents(normalizedStudents);
      } catch (error) {
        console.error("Failed to load students:", error);
        setStudents([]);
      }
    }

    loadStudents();
  }, [apiClient]);

  useEffect(() => {
    if (lessons.length > 0 && !selectedLessonId) {
      setSelectedLessonId(lessons[0].id ?? lessons[0].Id);
    }
  }, [lessons, selectedLessonId]);

  useEffect(() => {
    if (students.length > 0 && !selectedStudentId) {
      setSelectedStudentId(students[0].id);
    }
  }, [students, selectedStudentId]);

  const selectedLesson = useMemo(
    () => mockLessons.find((l) => l.id === selectedLessonId),
    [selectedLessonId]
  );

  const studentsForLesson = students;

  const sttForLesson = mockSTT[selectedLessonId] || {};
  const headerStats = useMemo(() => {
    const totalLessons = lessons.length;
    const totalStudents = students.length;
    const accVals = Object.values(mockSTT).flatMap((obj) =>
      Object.values(obj).map((v) => v.accuracy)
    );
    const avgAcc = accVals.length
      ? accVals.reduce((a, b) => a + b, 0) / accVals.length
      : 0;

    return { totalLessons, totalStudents, avgAcc };
  }, [lessons, students]);

  function handleAddNote(e) {
    e.preventDefault();
    const form = e.currentTarget;
    const text = form.note.value.trim();
    if (!text) return;
    addNote(selectedStudentId, selectedLessonId, text);
    form.reset();
  }

  function goToStudent(student) {
      const id = student.id ?? student.Id;
      navigate(`/student/${id}`);
  }

  return (
    <DashboardLayout title="SLP Dashboard">
      <div className="grid gap-4 sm:grid-cols-3">
        <div className="rounded-lg bg-white p-4 shadow">
          <div className="text-sm text-gray-600">Lessons</div>
          <div className="mt-1 text-2xl font-semibold">{headerStats.totalLessons}</div>
        </div>
        <div className="rounded-lg bg-white p-4 shadow">
          <div className="text-sm text-gray-600">Students Assigned</div>
          <div className="mt-1 text-2xl font-semibold">{headerStats.totalStudents}</div>
        </div>
        <div className="rounded-lg hover:bg-blue-200 hover:cursor-pointer p-4 shadow" 
            onClick={()=>{
                setShowPairRobotCard(true);
              }}>
          <div className="text-sm text-gray-600">Robot Status</div>
          <div className="mt-1 flex items-center gap-2">
            <span
              className={`h-2.5 w-2.5 rounded-full ${
                isPaired ? "bg-green-500" : "bg-red-500"
              }`}

            />

            <span className="text-2xl font-semibold">
              {isPaired ? "Connected" : "Disconnected"}
            </span>
          </div>
        </div>
      </div>


      <div className="mt-6">
        <section className="rounded-lg bg-white p-4 shadow">
          <div className="flex items-center justify-between">
            <h3 className="text-base font-semibold">Lessons</h3>

            <button
              type="button"
              onClick={() => navigate("/lessons")}
              className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500"
            >
              View all
            </button>
          </div>

          <div className="mt-3 space-y-3">
            {lessons.map((lesson) => (
              <LessonCard
                key={lesson.id || lesson.Id}
                lesson={lesson}
                onClick={() => {
                  const lessonId = lesson.id ?? lesson.Id;
                  navigate(`/lesson/${lessonId}`);
                }}
              />
            ))}
          </div>
        </section>
      </div>

      <div className="mt-6">
        <section className="rounded-lg bg-white p-4 shadow">
          <div className="flex items-center justify-between">
            <h3 className="text-base font-semibold">Students</h3>
            <button
                type="button"
                onClick={() => navigate("/students")}
                className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500"
              >
                View all
            </button>
          </div>
          <div className="mt-3 space-y-3 mx-auto w-full">
            {studentsForLesson.map((s) => (
              <StudentCard
                key={s.id}
                name={s.fullName}
                email={s.email}
                active={[]}
                completed={[]}
                selected={selectedStudentId === s.id}
                // onClick={() => setSelectedStudentId(s.id)}
                onClick={() => goToStudent(s)}
              />
            ))}
          </div>
        </section>
      </div>

      {/* <div className="mt-6 grid gap-4 lg:grid-cols-2">
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Add Note</h3>
          <p className="mt-1 text-sm text-gray-600">
            Notes are stored locally for now and can be moved to your API later.
          </p>
          <form onSubmit={handleAddNote} className="mt-3 space-y-3">
            <div className="grid gap-3 sm:grid-cols-2">
              <div>
                <label className="text-xs text-gray-600">Lesson</label>
                <select
                  value={selectedLessonId ?? ""}
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
                  value={selectedStudentId ?? ""}
                  onChange={(e) => setSelectedStudentId(e.target.value)}
                  className="mt-1 block w-full rounded-md border-gray-300 text-sm"
                >
                  {students.map((s) => (
                    <option key={s.id} value={s.id}>{s.fullName}</option>
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
      </div> */}
      {showPairRobotCard && (
                      <div className="fixed inset-0 z-50 flex items-center justify-center">
                          <div
                          className="absolute inset-0 bg-black/40"
                          onClick={() => setShowPairRobotCard(false)}
                          />
      
                          <div className="relative z-10 w-full max-w-xl px-4">
                              <PairRobotCard
                                onCancel={() => setShowPairRobotCard(false)}
                                onPaired={() => setShowPairRobotCard(false)}
                                onUnpaired={() => setShowPairRobotCard(false)}
                              />
                          </div>
                      </div>
                  )}
    </DashboardLayout>
  );
}
