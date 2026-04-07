import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { LessonCard } from "./LessonCard";
import { useApiClient } from "../context/ApiClientContext";

export default function Lessons() {
    const navigate = useNavigate();
    const apiClient = useApiClient();
    const [lessons, setLessons] = useState([]);

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

    function goToLesson(lessonId) {
        navigate(`/lesson/${lessonId}`);
    }

    return (
        <div className="w-full rounded-lg border p-6 shadow-sm">
            <div className="mb-6">
                <h2 className="text-2xl font-bold tracking-tight text-gray-900">
                    Lessons
                </h2>
                <p className="mt-1 text-sm text-gray-600">
                    Explore and manage your lessons
                </p>
            </div>

            <div className="space-y-4">
                {lessons.map((lesson) => (
                    <LessonCard
                        key={lesson.id ?? lesson.Id}
                        title={lesson.title ?? lesson.Title}
                        description={lesson.description ?? lesson.Description ?? ""}
                        onClick={() => goToLesson(lesson.id ?? lesson.Id)}
                    />
                ))}
            </div>
        </div>
    );
}