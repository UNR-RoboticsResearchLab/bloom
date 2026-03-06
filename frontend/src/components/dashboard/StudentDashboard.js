import DashboardLayout from "./DashboardLayout";
import { getSession } from "../../utils/auth";

const sampleByEmail = {
  // Map email to mock data if you want to demo different accounts
  "student@example.com": { active: ["R sound practice"], completed: ["S blends"] },
};

const defaultStudentData = { active: ["R sound practice"], completed: ["Breath control"] };

export default function StudentDashboard() {
  const session = getSession();
  const data =
    (session?.email && sampleByEmail[session.email.toLowerCase()]) || defaultStudentData;

  return (
    <DashboardLayout title="Student Dashboard">
      <div className="grid gap-4 lg:grid-cols-2">
        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Active Lessons</h3>
          <ul className="mt-3 list-disc pl-5 text-sm text-gray-700">
            {data.active.length ? data.active.map((l, i) => <li key={i}>{l}</li>) : <li>None</li>}
          </ul>
        </section>

        <section className="rounded-lg bg-white p-4 shadow">
          <h3 className="text-base font-semibold">Completed Lessons</h3>
          <ul className="mt-3 list-disc pl-5 text-sm text-gray-700">
            {data.completed.length ? data.completed.map((l, i) => <li key={i}>{l}</li>) : <li>None</li>}
          </ul>
        </section>
      </div>

      <section className="mt-6 rounded-lg bg-white p-4 shadow">
        <h3 className="text-base font-semibold">Notes</h3>
        <p className="mt-2 text-sm text-gray-600">
          Your therapist or teacher can add notes to each lesson. Once your API is connected,
          show the notes related to your account here.
        </p>
      </section>
    </DashboardLayout>
  );
}
