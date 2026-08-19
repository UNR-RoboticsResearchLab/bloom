import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { useApiClient } from "../context/ApiClientContext";
import { getSession } from "../utils/auth";
import LessonList from "../components/LessonList";
import LessonBuilder from "../components/LessonBuilder";

export default function BrowseLessons() {
    const apiClient = useApiClient();
    const navigate = useNavigate();
    const [lessons, setLessons] = useState([]);
    const [loading, setLoading] = useState(true);
    const [showCreate, setShowCreate] = useState(false);
    const isLoggedIn = Boolean(getSession()?.id);

    useEffect(() => {
        async function loadLessons() {
            try {
                const data = await apiClient.getLessons();
                setLessons(Array.isArray(data) ? data : []);
            } catch (error) {
                console.error("Failed to load lessons:", error);
                setLessons([]);
            } finally {
                setLoading(false);
            }
        }
        loadLessons();
    }, [apiClient]);

    async function handleCreateSubmit(dto) {
        const result = await apiClient.createLesson(dto);
        setShowCreate(false);
        const created = result?.lesson ?? result;
        navigate(`/lesson/${created?.id ?? created?.Id}`);
    }

    return (
        <main className="max-w-4xl mx-auto space-y-6 pb-8">
            <div className="flex items-start justify-between gap-4">
                <div>
                    <h1 className="text-2xl font-bold text-gray-900">Browse Lessons</h1>
                    <p className="mt-1 text-sm text-gray-500">
                        Discover lessons created by educators on the Bloom platform
                    </p>
                </div>
                {isLoggedIn && (
                    <button
                        onClick={() => setShowCreate(true)}
                        className="inline-flex shrink-0 items-center gap-1.5 rounded-md bg-indigo-600 px-4 py-2 text-sm font-semibold text-white shadow-sm hover:bg-indigo-500"
                    >
                        + Create Lesson
                    </button>
                )}
            </div>
            <LessonList lessons={lessons} loading={loading} />

            {isLoggedIn && showCreate && (
                <div className="fixed inset-0 z-40 flex items-center justify-center p-4">
                    <div
                        className="absolute inset-0 bg-black/30"
                        onClick={() => setShowCreate(false)}
                    />
                    <div className="relative z-50 w-full max-w-7xl max-h-[90vh] overflow-y-auto bg-gray-50 shadow-xl rounded-xl">
                        <div className="flex items-center justify-between border-b border-gray-200 bg-white px-6 py-4">
                            <h2 className="text-lg font-semibold text-gray-900">New Lesson</h2>
                            <button
                                onClick={() => setShowCreate(false)}
                                className="text-gray-400 hover:text-gray-600 text-xl leading-none"
                            >
                                &times;
                            </button>
                        </div>
                        <LessonBuilder
                            onSubmit={handleCreateSubmit}
                            onCancel={() => setShowCreate(false)}
                        />
                    </div>
                </div>
            )}
        </main>
    );
}
