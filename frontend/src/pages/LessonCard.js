const TYPE_CONFIG = {
    0: { label: "Language", badge: "bg-blue-100 text-blue-700", border: "border-l-blue-400" },
    1: { label: "Speech Therapy", badge: "bg-violet-100 text-violet-700", border: "border-l-violet-400" },
};

export function LessonCard({ lesson, onClick }) {
    const typeKey = lesson?.lessonType ?? lesson?.LessonType ?? 0;
    const config = TYPE_CONFIG[typeKey] ?? TYPE_CONFIG[0];
    const title = lesson?.title ?? lesson?.Title ?? "Untitled";
    const description = lesson?.description ?? lesson?.Description ?? "";
    const stepCount = lesson?.totalSteps ?? lesson?.TotalSteps ?? 0;
    const objectives = lesson?.learningObjectives ?? lesson?.LearningObjectives ?? [];

    return (
        <button
            onClick={onClick}
            className={`w-full text-left rounded-xl border border-gray-200 border-l-4 ${config.border} bg-white p-5 shadow-sm hover:shadow-md transition-all group focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-1`}
            aria-label={`View lesson: ${title}`}
        >
            <div className="flex items-start justify-between gap-4">
                <p className="text-base font-semibold text-gray-900 group-hover:text-indigo-600 transition-colors">
                    {title}
                </p>
                <span className={`shrink-0 rounded-full px-2.5 py-0.5 text-xs font-medium ${config.badge}`}>
                    {config.label}
                </span>``
            </div>
            {description && (
                <p className="mt-1.5 text-sm text-gray-500 line-clamp-2">{description}</p>
            )}
            <div className="mt-3 flex items-center gap-4 text-xs text-gray-400">
                {stepCount > 0 && <span>{stepCount} steps</span>}
                {Array.isArray(objectives) && objectives.length > 0 && (
                    <span>{objectives.length} objective{objectives.length !== 1 ? "s" : ""}</span>
                )}
            </div>
        </button>
    );
}