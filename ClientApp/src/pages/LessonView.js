import { useLocation, useNavigate } from "react-router-dom";
import { useEffect } from "react";

export default function LessonView() {
    const location = useLocation();
    const navigate = useNavigate();

    const { lesson, student } = location.state || {};

    useEffect(() => {
        if (!lesson || !student) {
            navigate("/lessons");
        }
    }, [lesson, student, navigate]);

    if (!lesson || !student) {
        return null;
    }

    return (
        <div className="rounded-lg border p-6 shadow-sm">
            <p className="text-sm font-semibold text-gray-900">
                Lesson Started
            </p>

            <div className="mt-4 rounded-lg border p-4 shadow-sm">
                <h1 className="text-3xl font-semibold text-gray-900">
                    {lesson?.title ?? lesson?.Title}
                </h1>

                <p className="mt-2 text-sm text-gray-600">
                    {lesson?.description ?? lesson?.Description ?? "No description available."}
                </p>
            </div>

            <div className="mt-6 rounded-lg border p-4 shadow-sm">
                <h2 className="text-lg font-semibold text-gray-900">Student</h2>
                <p className="mt-2 text-sm text-gray-700">
                    {student.firstName} {student.lastName}
                </p>
                <p className="text-sm text-gray-600">
                    Level: {student.level}
                </p>
            </div>

            <div className="mt-6 rounded-lg border p-4 shadow-sm">
                <h2 className="text-lg font-semibold text-gray-900">Lesson Session</h2>
                <p className="mt-2 text-sm text-gray-600">
                    The lesson has started for this student.
                </p>
            </div>
        </div>
    );
}