import { useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";
import LessonList from "../components/LessonList";

export default function BrowseLessons() {
    const apiClient = useApiClient();
    const [lessons, setLessons] = useState([]);
    const [loading, setLoading] = useState(true);

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

    return (
        <main className="max-w-4xl mx-auto space-y-6 pb-8">
            <div>
                <h1 className="text-2xl font-bold text-gray-900">Browse Lessons</h1>
                <p className="mt-1 text-sm text-gray-500">
                    Discover lessons created by educators on the Bloom platform
                </p>
            </div>
            <LessonList lessons={lessons} loading={loading} />
        </main>
    );
}
