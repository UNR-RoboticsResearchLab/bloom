import { useNavigate } from "react-router-dom";
import { useEffect, useState } from "react";
import { StudentCard } from "./StudentCard";
import AddStudentCard from "./AddStudentCard";
import { useApiClient } from "../context/ApiClientContext";

export default function Students() {
    const navigate = useNavigate();
    const api = useApiClient();

    const [students, setStudents] = useState([]);
    const [showAddStudentCard, setShowAddStudentCard] = useState(false);

    function goToStudent(student) {
        const id = student.id ?? student.Id;
        navigate(`/student/${id}`);
    }

    useEffect(() => {
        async function loadStudents() {
            try {
                const data = await api.getStudents();
                console.log("Students from backend:", data);
                setStudents(Array.isArray(data) ? data : []);
            } catch (err) {
                console.error("Failed to load students:", err);
                setStudents([]);
            }
        }

        loadStudents();
    }, [api]);

    function normalizeStudent(client) {
        return {
            id: client.studentId,
            fullName: client.studentName,
            email: client.email ?? "N/A"
        };
    }

    return (
        <div className="w-full rounded-lg border p-6 shadow-sm">
            <div className="mb-6 flex items-start justify-between gap-4">
                <div>
                    <h2 className="text-2xl font-bold tracking-tight text-gray-900">
                        Students
                    </h2>
                    <p className="mt-1 text-sm text-gray-600">
                        Manage and track your students progress
                    </p>
                </div>

                <button
                    type="button"
                    onClick={() => setShowAddStudentCard(true)}
                    className="rounded-md bg-indigo-600 px-3 py-1.5 text-sm font-semibold text-white hover:bg-indigo-500"
                >
                    Add Student
                </button>
            </div>

            <div className="space-y-4">
                {students.length === 0 && (
                    <p className="text-sm text-gray-500">No students found</p>
                )}

                {students.map((client) => {
                    const student = normalizeStudent(client);

                    return (
                        <StudentCard
                            key={student.id}
                            name={student.fullName}
                            email={student.email}
                            onClick={() => goToStudent(student)}
                        />
                    );
                })}
            </div>

            {showAddStudentCard && (
                <div className="fixed inset-0 z-50 flex items-center justify-center">
                    <div
                        className="absolute inset-0 bg-black/40"
                        onClick={() => setShowAddStudentCard(false)}
                    />

                    <div className="relative z-10 w-full max-w-xl px-4">
                        <AddStudentCard onCancel={() => setShowAddStudentCard(false)} />
                    </div>
                </div>
            )}
        </div>
    );
}