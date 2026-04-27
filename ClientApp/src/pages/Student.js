import { useEffect, useState } from "react";
import { useNavigate, useParams } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";

function getInitials(name) {
    return name
        .split(" ")
        .map(word => word[0])
        .join("")
        .toUpperCase();
}

export default function Student() {
    const { studentId } = useParams();
    const api = useApiClient();
    const navigate = useNavigate();

    const [student, setStudent] = useState(null);
    const [isLoading, setIsLoading] = useState(true);
    const [lessonHistory, setLessonHistory] = useState([]);
    const [isLoadingHistory, setIsLoadingHistory] = useState(true);

    useEffect(() => {
        async function loadStudent() {
            try {
                const data = await api.getStudent(studentId);
                console.log("Student from backend:", data);
                setStudent(data);
            } catch (err) {
                console.error("Failed to load student:", err);
                setStudent(null);
            } finally {
                setIsLoading(false);
            }
        }

        loadStudent();
    }, [api, studentId]);

    useEffect(() => {
        async function loadLessonHistory() {
            try {
                const data = await api.getStudentLessonHistory(studentId);
                console.log("Lesson history from backend:", data);
                setLessonHistory(Array.isArray(data) ? data : []);
            } catch (err) {
                console.error("Failed to load lesson history:", err);
                setLessonHistory([]);
            } finally {
                setIsLoadingHistory(false);
            }
        }

        loadLessonHistory();
    }, [api, studentId]);

    if (isLoading) {
        return <div className="rounded-lg border p-6 shadow-sm">Loading student...</div>;
    }

    if (!student) {
        return <div className="rounded-lg border p-6 shadow-sm">Student not found</div>;
    }

    const name = student.fullName ?? student.studentName ?? student.name ?? "No Name";
    const initials = getInitials(name);
    
    return (
        <div className= "rounded-lg border p-6 shadow-sm">
            <p className=" text-sm font-semibold text-gray-900">
                Students &gt; {name}
            </p>

            <div className=" mt-3 rounded-lg border p-4 shadow-sm">
                <div className=" flex items-center gap-4 ">
                    <div className=" inline-flex h-20 w-20 items-center justify-center rounded-full border border-gray-900 bg-white">
                        <span className=" text-4xl font-semibold text-gray-900">
                            {initials}
                        </span>
                    </div>

                    <div className="leading-tight">
                        <p className="text-sm font-semibold text-gray-900">
                            {name}
                        </p>
                        <p className="text-xs text-gray-900">
                            level 2
                        </p>
                        <p className="text-xs text-gray-900">
                            Active
                        </p>
                    </div>
                </div>
            </div>

            <div className="mt-4 rounded-lg border p-6 shadow-sm">
                <p className="text-base font-semibold text-gray-900">Notes</p>
                <p className="mt-1 text-sm text-gray-500">
                    Record observations and recommendations for this student.
                </p>

                <div className="mt-6 border-t border-gray-200 pt-6">
                    <p className="text-sm font-semibold text-gray-900">Add a new note</p>
                    <textarea
                    id="note"
                    className="mt-3 w-full rounded-lg border border-gray-300 p-3 text-sm shadow-sm focus:border-indigo-500 focus:ring-2 focus:ring-indigo-200"
                    rows={4}
                    placeholder="Type your note here"
                    />
                    <div className="mt-4 flex justify-end">
                        <button className="rounded-md bg-indigo-600 px-4 py-2 text-sm font-medium text-white shadow-sm hover:bg-indigo-500 focus:outline-none focus:ring-2 focus:ring-indigo-400">
                            Save note
                        </button>
                    </div>
                </div>
            </div>

            <div className="mt-6 rounded-lg border shadow-sm">  
                <div className="divide-y">
                    {isLoadingHistory ? (
                        <div className="px-6 py-4">
                            <p className="text-sm text-gray-500">Loading lesson history...</p>
                        </div>
                    ) : lessonHistory.length === 0 ? (
                        <div className="px-6 py-4">
                            <p className="text-sm text-gray-500">No past sessions found.</p>
                        </div>
                    ) : (
                        lessonHistory.map((item, index) => {
                            const sessionId = item.sessionId ?? item.SessionId ?? item.id ?? item.Id;
                            const lessonTitle = item.lessonTitle ?? item.LessonTitle ?? item.title ?? "Untitled Lesson";
                            const date = item.startedAt ?? item.StartedAt ?? item.createdAt ?? item.CreatedAt;

                            return (
                                <button
                                    key={sessionId ?? index}
                                    type="button"
                                    onClick={() => navigate(`/lesson-history/${sessionId}`)}
                                    className="block w-full px-6 py-4 text-left hover:bg-gray-50"
                                >
                                    <p className="text-sm font-medium text-gray-900">
                                        {lessonTitle}
                                    </p>
                                    <p className="mt-1 text-sm text-gray-500">
                                        {date ? new Date(date).toLocaleDateString() : "No date"}
                                    </p>
                                </button>
                            );
                        })
                    )}
                </div>
            </div>
        </div>
    );
}