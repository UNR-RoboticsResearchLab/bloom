import { LessonCard } from './LessonCard';

export default function Lessons() {
    return (
        <div>
            <div className="">
                <div className="space-y-4">
                    <LessonCard title="Lesson 1" description="Introduction to Bloom" onClick={() => alert("Clicked Lesson 1")} />
                </div>
            </div>
            <div className="">
                <div className="space-y-4">
                    <LessonCard title="Lesson 2" description="Using Bloom for SLPs" onClick={() => alert("Clicked Lesson 2")} />
                </div>
            </div>
            <div className="">
                <div className="space-y-4">
                    <LessonCard title="Lesson 3" description="Advanced Features of Bloom" onClick={() => alert("Clicked Lesson 3")} />
                </div>
            </div>
        </div>
        
    );
}