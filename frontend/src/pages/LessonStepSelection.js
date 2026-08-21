import { useLocation } from "react-router-dom";

export default function LessonStepSelection() {
    const location = useLocation();
    const { lesson, student, sessionId, lessonId } = location.state || {};

    return (
        <div className="mt-4 rounded-lg border p-6 shadow-sm">
            <p className="text-sm font-semibold text-gray-900">
                Lesson In Progress
            </p>

            <div className="mt-4 rounded-lg border p-4 shadow-sm">
                <h1 className="text-3xl font-semibold text-gray-900">
                    {lesson?.title ?? lesson?.Title}
                </h1>

                <p className="mt-2 text-sm text-gray-600">
                    {lesson?.description ?? lesson?.Description ?? "No description available."}
                </p>

                <div className="mt-4 grid gap-3 md:grid-cols-3 text-sm">
                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Student</p>
                        <p className="mt-1 text-gray-700">
                            {student.firstName} {student.lastName}
                        </p>
                        <p className="text-gray-600">Level: {student.level}</p>
                    </div>

                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Session</p>
                        <p className="mt-1 break-all text-gray-600">
                            {sessionId || "No paired session found"}
                        </p>
                    </div>

                    <div className="rounded-md border p-3">
                        <p className="font-semibold text-gray-900">Lesson ID</p>
                        <p className="mt-1 break-all text-gray-600">
                            {lessonId}
                        </p>
                    </div>
                </div>
            </div> 
        </div>
    );
}
