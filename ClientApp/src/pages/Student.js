import { useEffect, useState } from "react";
import { useParams } from "react-router-dom";
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

    const [student, setStudent] = useState(null);
    const [isLoading, setIsLoading] = useState(true);

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
                
                <div className="border-b px-6 py-4">
                    <p className="text-base font-semibold text-gray-900">Lesson History</p>
                    <p className="mt-1 text-sm text-gray-500">Past sessions</p>
                </div>

                
                <div className="border-b px-6 py-4">
                    <p className="text-sm font-medium text-gray-900">
                    Articulation Practice: /r/ sounds
                    </p>
                    <p className="mt-1 text-sm text-gray-500">
                    February 17, 2026 | 30 minutes
                    </p>
                </div>

                
                <div className="px-6 py-4">
                    <p className="text-sm font-medium text-gray-900">
                    Articulation Practice: /s/ sounds
                    </p>
                    <p className="mt-1 text-sm text-gray-500">
                    February 15, 2026 | 30 minutes
                    </p>
                </div>
            </div>
        </div>
    );
}

