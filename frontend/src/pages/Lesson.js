import { useParams, useNavigate } from "react-router-dom";
import { useEffect, useState } from "react";
import { useApiClient } from "../context/ApiClientContext";
import { getSession } from "../utils/auth";
import SelectStudentCard from "./SelectStudentCard";
import LessonBuilder from "../components/LessonBuilder";

const TYPE_CONFIG = {
    0: { label: "Language", bg: "bg-blue-50", badge: "bg-blue-100 text-blue-700", accent: "bg-blue-500" },
    1: { label: "Speech Therapy", bg: "bg-violet-50", badge: "bg-violet-100 text-violet-700", accent: "bg-violet-500" },
};

function StepRow({ step, index }) {
    const [open, setOpen] = useState(false);

    const stepTitle = step.title ?? step.Title;
    const stepType  = step.type  ?? step.Type;
    const script    = step.script ?? step.Script;
    const timing    = step.timingSeconds ?? step.TimingSeconds;
    const visualAid = step.visualAid ?? step.VisualAid;
    const visualAidLabels   = step.visualAidLabels   ?? step.VisualAidLabels;
    const visualAidFooters  = step.visualAidFooters  ?? step.VisualAidFooters;
    const motorSequence     = step.motorSequence     ?? step.MotorSequence;
    const behaviors  = step.behaviors  ?? step.Behaviors;
    const interaction = step.interaction ?? step.Interaction;

    const props = [
        script           && { label: "Script",           value: script },
        timing != null   && { label: "Timing",           value: `${timing}s` },
        visualAid        && { label: "Visual Aid",        value: visualAid },
        visualAidLabels  && { label: "Visual Aid Labels", value: visualAidLabels },
        visualAidFooters && { label: "Visual Aid Footers",value: visualAidFooters },
        motorSequence    && { label: "Motor Sequence",    value: motorSequence },
        behaviors?.behavior         && { label: "Behavior",         value: behaviors.behavior },
        behaviors?.facialExpression && { label: "Facial Expression", value: behaviors.facialExpression },
        behaviors?.gaze             && { label: "Gaze",             value: behaviors.gaze },
        behaviors?.headMovement     && { label: "Head Movement",    value: behaviors.headMovement },
    ].filter(Boolean);

    const interactionProps = interaction ? [
        interaction.waitForResponse  != null && { label: "Wait for Response",    value: String(interaction.waitForResponse) },
        interaction.maxWaitSeconds   != null && { label: "Max Wait",             value: `${interaction.maxWaitSeconds}s` },
        interaction.correctAnswer               && { label: "Correct Answer",         value: interaction.correctAnswer },
        interaction.correctResponseScript       && { label: "Correct Response",        value: interaction.correctResponseScript },
        interaction.incorrectResponseScript     && { label: "Incorrect Response",      value: interaction.incorrectResponseScript },
        interaction.singleTurnLlm    != null && { label: "Single-Turn LLM",      value: String(interaction.singleTurnLlm) },
        interaction.singleTurnLlmPrompt         && { label: "LLM Prompt",             value: interaction.singleTurnLlmPrompt },
        interaction.llmFollowUp      != null && { label: "LLM Follow-Up",        value: String(interaction.llmFollowUp) },
        interaction.fallbackScript              && { label: "Fallback Script",         value: interaction.fallbackScript },
    ].filter(Boolean) : [];

    return (
        <li key={step.id ?? step.Id ?? index} className="rounded-lg border border-gray-100 overflow-hidden">
            <button
                onClick={() => setOpen((o) => !o)}
                aria-expanded={open}
                className="w-full flex items-center gap-3 bg-gray-50 p-3 hover:bg-gray-100 transition-colors text-left"
            >
                <span className="flex-shrink-0 w-7 h-7 rounded-full bg-indigo-100 text-indigo-700 flex items-center justify-center text-xs font-bold" aria-label={`Step ${index + 1}`}>
                    {index + 1}
                </span>
                <span className="flex-1 text-sm font-medium text-gray-800">
                    {stepTitle || `Step ${index + 1}`}
                </span>
                {stepType && (
                    <span className="text-xs px-2 py-0.5 rounded-full bg-gray-200 text-gray-600 capitalize">
                        {stepType}
                    </span>
                )}
                <span className="text-gray-400 text-xs ml-1">{open ? "▲" : "▼"}</span>
            </button>

            {open && (
                <div className="border-t border-gray-100 bg-white px-4 py-3 space-y-4">
                    {props.length > 0 && (
                        <dl className="grid grid-cols-1 sm:grid-cols-2 gap-x-6 gap-y-2">
                            {props.map(({ label, value }) => (
                                <div key={label}>
                                    <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">{label}</dt>
                                    <dd className="mt-0.5 text-sm text-gray-800 whitespace-pre-wrap break-words">{value}</dd>
                                </div>
                            ))}
                        </dl>
                    )}

                    {interactionProps.length > 0 && (
                        <div>
                            <p className="text-xs font-semibold text-gray-400 uppercase tracking-wide mb-2">Interaction</p>
                            <dl className="grid grid-cols-1 sm:grid-cols-2 gap-x-6 gap-y-2 rounded-lg bg-gray-50 p-3">
                                {interactionProps.map(({ label, value }) => (
                                    <div key={label}>
                                        <dt className="text-xs font-semibold text-gray-400 uppercase tracking-wide">{label}</dt>
                                        <dd className="mt-0.5 text-sm text-gray-800 whitespace-pre-wrap break-words">{value}</dd>
                                    </div>
                                ))}
                            </dl>
                        </div>
                    )}

                    {props.length === 0 && interactionProps.length === 0 && (
                        <p className="text-sm text-gray-400 italic">No additional properties.</p>
                    )}
                </div>
            )}
        </li>
    );
}

export default function Lesson() {
    const { lessonId } = useParams();
    const apiClient = useApiClient();
    const navigate = useNavigate();

    const [lesson, setLesson] = useState(null);
    const [students, setStudents] = useState([]);
    const [showSelectStudent, setShowSelectStudent] = useState(false);
    const [showEdit, setShowEdit] = useState(false);

    const typeKey = lesson?.lessonType ?? lesson?.LessonType ?? 0;
    const typeConfig = TYPE_CONFIG[typeKey] ?? TYPE_CONFIG[0];
    const title = lesson?.title ?? lesson?.Title;
    const description = lesson?.description ?? lesson?.Description;
    const objectives = lesson?.learningObjectives ?? lesson?.LearningObjectives ?? [];
    const steps = lesson?.steps ?? lesson?.Steps ?? [];
    const totalSteps = lesson?.totalSteps ?? lesson?.TotalSteps ?? steps.length;
    const createdDate = lesson?.createdDate ?? lesson?.CreatedDate;
    const createdById = lesson?.createdById ?? lesson?.CreatedById;
    const session = getSession();
    const isOwner = Boolean(session?.id && createdById && session.id === createdById);

    function handleStudentSelect(student) {
        setShowSelectStudent(false);
        navigate("/lesson-view", { state: { lesson, student } });
    }

    function handleEditSubmit() {
        // TODO: wire up to a lesson update endpoint once the backend supports it.
        setShowEdit(false);
    }

    useEffect(() => {
        async function loadLesson() {
            try {
                const data = await apiClient.getLesson(lessonId);
                setLesson(data);
            } catch (err) {
                console.error("Failed loading lesson", err);
            }
        }
        loadLesson();
    }, [lessonId, apiClient]);

    useEffect(() => {
        async function loadStudents() {
            try {
                const data = await apiClient.getStudents();
                const normalized = Array.isArray(data)
                    ? data.map((c) => ({ id: c.studentId, fullName: c.studentName, email: c.email ?? "N/A" }))
                    : [];
                setStudents(normalized);
            } catch (err) {
                setStudents([]);
            }
        }
        loadStudents();
    }, [apiClient]);

    if (!lesson) {
        return (
            <div className="flex items-center justify-center min-h-64" role="status" aria-live="polite">
                <div className="text-center space-y-3">
                    <div className="w-8 h-8 border-4 border-indigo-600 border-t-transparent rounded-full animate-spin mx-auto" aria-hidden="true" />
                    <p className="text-sm text-gray-500">Loading lesson...</p>
                </div>
            </div>
        );
    }

    return (
        <main className="max-w-3xl mx-auto space-y-6 pb-8">
            {/* Breadcrumb */}
            <nav aria-label="Breadcrumb" className="flex items-center justify-between">
                <button
                    onClick={() => navigate("/lessons")}
                    className="inline-flex items-center gap-1.5 text-sm text-gray-500 hover:text-indigo-600 transition-colors"
                >
                    ← Back to Lessons
                </button>
                {isOwner && (
                    <button
                        onClick={() => setShowEdit(true)}
                        className="inline-flex items-center gap-1.5 rounded-md border border-gray-300 bg-white px-3 py-1.5 text-sm font-medium text-gray-700 shadow-sm hover:bg-gray-50"
                    >
                        Edit Lesson
                    </button>
                )}
            </nav>

            {/* Hero */}
            <header className={`rounded-2xl ${typeConfig.bg} border border-gray-200 p-8 shadow-sm`}>
                <span className={`inline-flex items-center rounded-full px-3 py-1 text-xs font-semibold ${typeConfig.badge}`}>
                    {typeConfig.label}
                </span>
                <h1 className="mt-3 text-3xl md:text-4xl font-bold text-gray-900 leading-tight">
                    {title}
                </h1>
                {description && (
                    <p className="mt-3 text-base text-gray-600 leading-relaxed">{description}</p>
                )}
                <div className="mt-5 flex flex-wrap gap-4 text-sm text-gray-500">
                    <span>
                        <strong className="text-gray-800">{totalSteps}</strong> steps
                    </span>
                    {Array.isArray(objectives) && objectives.length > 0 && (
                        <span>
                            <strong className="text-gray-800">{objectives.length}</strong> learning objective{objectives.length !== 1 ? "s" : ""}
                        </span>
                    )}
                    {createdDate && (
                        <span>Created {new Date(createdDate).toLocaleDateString()}</span>
                    )}
                </div>
            </header>

            {/* Learning Objectives */}
            {Array.isArray(objectives) && objectives.length > 0 && (
                <section aria-labelledby="objectives-heading" className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
                    <h2 id="objectives-heading" className="text-lg font-semibold text-gray-900">
                        Learning Objectives
                    </h2>
                    <ul className="mt-4 space-y-2.5" role="list">
                        {objectives.map((obj, i) => (
                            <li key={i} className="flex items-start gap-3 text-sm text-gray-700">
                                <span
                                    className="mt-0.5 flex-shrink-0 w-5 h-5 rounded-full bg-green-100 text-green-700 flex items-center justify-center text-xs font-bold"
                                    aria-hidden="true"
                                >
                                    ✓
                                </span>
                                <span>{obj}</span>
                            </li>
                        ))}
                    </ul>
                </section>
            )}

            {/* Steps Preview */}
            {steps.length > 0 && (
                <section aria-labelledby="steps-heading" className="rounded-xl border border-gray-200 bg-white p-6 shadow-sm">
                    <h2 id="steps-heading" className="text-lg font-semibold text-gray-900">
                        Lesson Steps
                        <span className="ml-2 text-sm font-normal text-gray-400">({steps.length} total)</span>
                    </h2>
                    <ol className="mt-4 space-y-2" role="list">
                        {steps.map((step, i) => (
                            <StepRow key={step.id ?? step.Id ?? i} step={step} index={i} />
                        ))}
                    </ol>
                </section>
            )}

            {/* CTA */}
            <div className="flex justify-center pt-2">
                <button
                    onClick={() => setShowSelectStudent(true)}
                    className="rounded-xl bg-indigo-600 px-10 py-4 text-base font-semibold text-white shadow-md hover:bg-indigo-500 active:bg-indigo-700 transition-colors focus:outline-none focus:ring-2 focus:ring-indigo-500 focus:ring-offset-2"
                >
                    Start Lesson
                </button>
            </div>

            {showSelectStudent && (
                <div
                    className="fixed inset-0 flex items-center justify-center bg-black/40 z-50"
                    role="dialog"
                    aria-modal="true"
                    aria-label="Select a student to start the lesson"
                >
                    <SelectStudentCard
                        students={students}
                        onSelect={handleStudentSelect}
                        onCancel={() => setShowSelectStudent(false)}
                    />
                </div>
            )}

            {showEdit && (
                <div
                    className="fixed inset-0 z-50 overflow-y-auto bg-black/40 p-4 sm:p-8"
                    role="dialog"
                    aria-modal="true"
                    aria-label="Edit lesson"
                >
                    <div className="mx-auto max-w-7xl rounded-2xl bg-gray-50 shadow-xl">
                        <LessonBuilder
                            initialLesson={lesson}
                            onSubmit={handleEditSubmit}
                            onCancel={() => setShowEdit(false)}
                        />
                    </div>
                </div>
            )}
        </main>
    );
}