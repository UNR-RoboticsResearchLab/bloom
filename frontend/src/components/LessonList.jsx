import { useState } from "react";
import { useNavigate } from "react-router-dom";

const TYPE_CONFIG = {
    0: { label: "Language", badge: "bg-blue-100 text-blue-700", border: "border-l-blue-400" },
    1: { label: "Speech Therapy", badge: "bg-violet-100 text-violet-700", border: "border-l-violet-400" },
};

const TYPE_FILTERS = [
    { value: "all", label: "All Lessons" },
    { value: "0", label: "Language" },
    { value: "1", label: "Speech Therapy" },
];

function LessonCard({ lesson, onClick }) {
    const typeKey = lesson?.lessonType ?? lesson?.LessonType ?? 0;
    const config = TYPE_CONFIG[typeKey] ?? TYPE_CONFIG[0];
    const title = lesson?.title ?? lesson?.Title ?? "Untitled";
    const description = lesson?.description ?? lesson?.Description ?? "";
    const stepCount = lesson?.totalSteps ?? lesson?.TotalSteps ?? 0;
    const objectives = lesson?.learningObjectives ?? lesson?.LearningObjectives ?? [];
    const createdDate = lesson?.createdDate ?? lesson?.CreatedDate;

    return (
        <button
            onClick={onClick}
            className={`w-full text-left rounded-xl border border-gray-200 border-l-4 ${config.border} bg-white p-5 shadow-sm hover:shadow-md transition-all group focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1`}
            aria-label={`View lesson: ${title}`}
        >
            <div className="flex items-start justify-between gap-4">
                <div className="flex-1 min-w-0">
                    <p className="text-base font-semibold text-gray-900 group-hover:text-indigo-600 transition-colors">
                        {title}
                    </p>
                    {description && (
                        <p className="mt-1 text-sm text-gray-500 line-clamp-2">{description}</p>
                    )}
                </div>
                <span className={`shrink-0 rounded-full px-2.5 py-0.5 text-xs font-medium ${config.badge}`}>
                    {config.label}
                </span>
            </div>

            {Array.isArray(objectives) && objectives.length > 0 && (
                <ul className="mt-3 space-y-1" aria-label="Learning objectives">
                    {objectives.map((obj, i) => (
                        <li key={i} className="flex items-start gap-2 text-xs text-gray-600">
                            <span className="mt-0.5 flex-shrink-0 w-4 h-4 rounded-full bg-green-100 text-green-700 flex items-center justify-center text-xs font-bold" aria-hidden="true">✓</span>
                            <span>{obj}</span>
                        </li>
                    ))}
                </ul>
            )}

            <div className="mt-3 flex items-center gap-4 text-xs text-gray-400">
                {stepCount > 0 && <span>{stepCount} steps</span>}
                {Array.isArray(objectives) && objectives.length > 0 && (
                    <span>{objectives.length} objective{objectives.length !== 1 ? "s" : ""}</span>
                )}
                {createdDate && (
                    <span className="ml-auto">{new Date(createdDate).toLocaleDateString()}</span>
                )}
            </div>
        </button>
    );
}

export default function LessonList({ lessons = [], loading = false }) {
    const navigate = useNavigate();
    const [search, setSearch] = useState("");
    const [typeFilter, setTypeFilter] = useState("all");

    const filtered = lessons.filter((l) => {
        const title = (l.title ?? l.Title ?? "").toLowerCase();
        const desc = (l.description ?? l.Description ?? "").toLowerCase();
        const objText = (l.learningObjectives ?? l.LearningObjectives ?? []).join(" ").toLowerCase();
        const q = search.toLowerCase();
        const matchSearch = !q || title.includes(q) || desc.includes(q) || objText.includes(q);
        const lessonType = String(l.lessonType ?? l.LessonType ?? 0);
        const matchType = typeFilter === "all" || lessonType === typeFilter;
        return matchSearch && matchType;
    });

    const languageCount = lessons.filter((l) => (l.lessonType ?? l.LessonType) === 0).length;
    const speechCount = lessons.filter((l) => (l.lessonType ?? l.LessonType) === 1).length;

    return (
        <div className="space-y-6">
            {!loading && (
                <div className="grid grid-cols-3 gap-4">
                    <div className="rounded-xl border border-gray-200 bg-white p-4 text-center shadow-sm">
                        <p className="text-2xl font-bold text-gray-900">{lessons.length}</p>
                        <p className="mt-0.5 text-xs text-gray-500">Total Lessons</p>
                    </div>
                    <div className="rounded-xl border border-blue-200 bg-blue-50 p-4 text-center shadow-sm">
                        <p className="text-2xl font-bold text-blue-700">{languageCount}</p>
                        <p className="mt-0.5 text-xs text-blue-500">Language</p>
                    </div>
                    <div className="rounded-xl border border-violet-200 bg-violet-50 p-4 text-center shadow-sm">
                        <p className="text-2xl font-bold text-violet-700">{speechCount}</p>
                        <p className="mt-0.5 text-xs text-violet-500">Speech Therapy</p>
                    </div>
                </div>
            )}

            <div className="flex flex-col sm:flex-row gap-3">
                <input
                    type="search"
                    value={search}
                    onChange={(e) => setSearch(e.target.value)}
                    placeholder="Search by title, description, or objective..."
                    aria-label="Search lessons"
                    className="flex-1 rounded-lg border border-gray-300 px-4 py-2 text-sm focus:outline-none focus:ring-2 focus:ring-indigo-500"
                />
                <div role="group" aria-label="Filter by lesson type" className="flex gap-1 rounded-lg border border-gray-200 bg-gray-50 p-1">
                    {TYPE_FILTERS.map((f) => (
                        <button
                            key={f.value}
                            onClick={() => setTypeFilter(f.value)}
                            aria-pressed={typeFilter === f.value}
                            className={`rounded-md px-3 py-1.5 text-xs font-medium transition-colors whitespace-nowrap ${
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

            {!loading && search && (
                <p className="text-sm text-gray-500" aria-live="polite">
                    {filtered.length} result{filtered.length !== 1 ? "s" : ""} for &ldquo;{search}&rdquo;
                </p>
            )}

            {loading ? (
                <div className="flex items-center justify-center py-20" role="status" aria-live="polite">
                    <div className="text-center space-y-3">
                        <div className="w-8 h-8 border-4 border-indigo-600 border-t-transparent rounded-full animate-spin mx-auto" aria-hidden="true" />
                        <p className="text-sm text-gray-500">Loading lessons...</p>
                    </div>
                </div>
            ) : filtered.length === 0 ? (
                <div className="rounded-xl border border-dashed border-gray-300 p-20 text-center text-gray-400">
                    <p className="text-sm">
                        {search ? `No lessons matching "${search}"` : "No lessons available yet"}
                    </p>
                </div>
            ) : (
                <div className="grid grid-cols-1 md:grid-cols-2 gap-4" role="list" aria-label="Available lessons">
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
        </div>
    );
}
