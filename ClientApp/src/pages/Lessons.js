import { useNavigate } from "react-router-dom";
import { LessonCard } from './LessonCard';

export default function Lessons() {
    const navigate = useNavigate();

    function goToLesson(name) {
        navigate(`/lesson/${encodeURIComponent(name)}`);
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
            <div className="mt-3">
                <div className="space-y-4">
                    <LessonCard title="Lesson 1" description="Introduction to Bloom" onClick={() => goToLesson("Lesson 1")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <LessonCard title="Lesson 2" description="Using Bloom for SLPs" onClick={() => goToLesson("Lesson 2")} />
                </div>
            </div>
            <div className="mt-3">
                <div className="space-y-4">
                    <LessonCard title="Lesson 3" description="Advanced Features of Bloom" onClick={() => goToLesson("Lesson 3")} />
                </div>
            </div>
        </div>  
    );
}