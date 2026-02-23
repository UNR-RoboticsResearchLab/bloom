import { useNavigate } from "react-router-dom";
import { useState } from "react";
import { StudentCard } from "./StudentCard";
import AddStudentCard from "./AddStudentCard";

export default function Students() {
    const navigate = useNavigate();
    const [showAddStudentCard, setShowAddStudentCard] = useState(false);

    function goToStudent(name) {
        navigate(`/student/${encodeURIComponent(name)}`);
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

            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="John Doe" email="john.doe@example.com" onClick={() => goToStudent("John Doe")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="Jane Smith" email="jane.smith@example.com" onClick={() => goToStudent("Jane Smith")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="Alice Johnson" email="alice.johnson@example.com" onClick={() => goToStudent("Alice Johnson")} />
                </div>
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