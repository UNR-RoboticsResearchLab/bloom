import {StudentCard} from "./StudentCard";

export default function Students() {
    return (
        <div className="w-full">
            <div className="mb-6">
                <h2 className="text-2xl font-bold tracking-tight text-gray-900">
                Students
                </h2>
                <p className="mt-1 text-sm text-gray-600">
                Manage and track your students progress
                </p>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="John Doe" email="john.doe@example.com" onClick={() => alert("Student clicked!")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="Jane Smith" email="jane.smith@example.com" onClick={() => alert("Student clicked!")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <StudentCard name="Alice Johnson" email="alice.johnson@example.com" onClick={() => alert("Student clicked!")} />
                </div>
            </div>
        </div>
    );
}