import { useEffect, useState } from "react";
import DashboardLayout from "./DashboardLayout";
import { CustomizeRobotCard } from "../../pages/CustomizeRobotCard";
import { useApiClient } from "../../context/ApiClientContext";
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

  const api = useApiClient();
  const [robotProfile, setRobotProfile] = useState(null);
  const [showCustomizeCard, setShowCustomizeCard] = useState(false);
  const [isLoadingProfile, setIsLoadingProfile] = useState(true);

  useEffect(() => {
    async function loadProfile() {
      try {
        const profile = await api.getMyRobotProfile();
        setRobotProfile(profile);
      } catch (err) {
        // No profile set up yet — treat as empty state rather than an error.
        setRobotProfile(null);
      } finally {
        setIsLoadingProfile(false);
      }
    }
    loadProfile();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, []);

  function handleProfileSaved(profile) {
    setRobotProfile(profile);
    setShowCustomizeCard(false);
  }

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
        <div className="flex items-center justify-between">
          <h3 className="text-base font-semibold">Your Robot</h3>
          <button
            type="button"
            onClick={() => setShowCustomizeCard(true)}
            className="rounded-md border border-gray-900 bg-indigo-600 px-3 py-1.5 text-sm font-medium text-white shadow-sm hover:bg-indigo-400"
          >
            {robotProfile ? "Edit" : "Customize Your Robot"}
          </button>
        </div>

        {!isLoadingProfile && (
          <div className="mt-3 text-sm text-gray-700">
            {robotProfile ? (
              <div className="flex items-center gap-3">
                <span
                  className="h-3 w-3 rounded-full border border-gray-300"
                  style={{ backgroundColor: robotProfile.colorTheme || "#6366f1" }}
                />
                <span>
                  <span className="font-medium">{robotProfile.nickname}</span>
                  {" — "}
                  {robotProfile.personalityTrait}
                  {robotProfile.catchphrase ? ` · "${robotProfile.catchphrase}"` : ""}
                </span>
              </div>
            ) : (
              <p className="text-gray-500">
                You haven't named your robot yet. Give it a name and personality!
              </p>
            )}
          </div>
        )}

        {showCustomizeCard && (
          <div className="mt-4">
            <CustomizeRobotCard
              profile={robotProfile}
              onCancel={() => setShowCustomizeCard(false)}
              onSaved={handleProfileSaved}
            />
          </div>
        )}
      </section>

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
