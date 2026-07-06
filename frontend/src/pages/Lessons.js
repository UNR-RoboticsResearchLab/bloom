import { useEffect, useState } from "react";
import { useNavigate } from "react-router-dom";
import { LessonCard } from "./LessonCard";
import { useApiClient } from "../context/ApiClientContext";

const TYPE_FILTERS = [
    { value: "all", label: "All" },
    { value: "0", label: "Language" },
    { value: "1", label: "Speech Therapy" },
];

export default function Lessons() {
    const navigate = useNavigate();
    const apiClient = useApiClient();
    const [lessons, setLessons] = useState([]);
    const [search, setSearch] = useState("");
    const [typeFilter, setTypeFilter] = useState("all");
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

    const filtered = lessons.filter((l) => {
        const title = (l.title ?? l.Title ?? "").toLowerCase();
        const desc = (l.description ?? l.Description ?? "").toLowerCase();
        const q = search.toLowerCase();
        const matchSearch = !q || title.includes(q) || desc.includes(q);
        const lessonType = String(l.lessonType ?? l.LessonType ?? 0);
        const matchType = typeFilter === "all" || lessonType === typeFilter;
        return matchSearch && matchType;
    });

    return (
        <main className="max-w-3xl mx-auto space-y-6 pb-8">
            <div>
                <h1 className="text-2xl font-bold text-gray-900">Lessons</h1>
                <p className="mt-1 text-sm text-gray-500">Browse and start lessons for your students</p>
            </div>

            {/* Search + type filter */}
            <div className="flex flex-col sm:flex-row gap-3">
                <input
                    type="search"
                    value={search}
                    onChange={(e) => setSearch(e.target.value)}
                    placeholder="Search lessons..."
                    aria-label="Search lessons"
                    className="flex-1 rounded-lg border border-gray-300 px-4 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-500"
                />
                <div role="group" aria-label="Filter by type" className="flex gap-1 rounded-lg border border-gray-200 bg-gray-50 p-1">
                    {TYPE_FILTERS.map((f) => (
                        <button
                            key={f.value}
                            onClick={() => setTypeFilter(f.value)}
                            aria-pressed={typeFilter === f.value}
                            className={`rounded-md px-3 py-1.5 text-xs font-medium transition-colors ${
                                typeFilter === f.value
                                    ? "bg-white shadow text-indigo-600"
                                    : "text-gray-500 hover:text-gray-700"
                            }`}
                        >
                            {f.label}
                        </button>
                    ))}
                </div>
            </div>

            {/* Results */}
            {loading ? (
                <div className="flex items-center justify-center py-16" role="status" aria-live="polite">
                    <div className="text-center space-y-3">
                        <div className="w-8 h-8 border-4 border-indigo-600 border-t-transparent rounded-full animate-spin mx-auto" aria-hidden="true" />
                        <p className="text-sm text-gray-500">Loading lessons...</p>
                    </div>
                </div>
            ) : filtered.length === 0 ? (
                <div className="rounded-xl border border-dashed border-gray-300 p-16 text-center text-gray-400">
                    <p className="text-sm">{search ? `No lessons matching "${search}"` : "No lessons available"}</p>
                </div>
            ) : (
                <div className="space-y-3" role="list" aria-label="Lessons">
                    {filtered.map((lesson) => (
                        <div role="listitem" key={lesson.id ?? lesson.Id}>
                            <LessonCard
                                lesson={lesson}
                                onClick={() => navigate(`/lesson/${lesson.id ?? lesson.Id}`)}
                            />
                        </div>
                    ))}
                </div>
            )}
        </main>
    );
}