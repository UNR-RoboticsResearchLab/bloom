export default class ApiClient {
  constructor(baseUrl) {
    this.baseUrl = baseUrl;
    console.log("API Baseurl:" + baseUrl);
  }

  async request(endpoint, options = {}) {
    const url = `${this.baseUrl}${endpoint}`;

    const headers = {
      "Content-Type": "application/json",
      ...(options.headers || {}),
    };

    const response = await fetch(url, {
      credentials: "include",
      ...options,
      headers,
    });

    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`HTTP ${response.status}: ${errorText}`);
    }

    try {
      return await response.json();
    } catch {
      return null;
    }
  }

  // ── Auth ──────────────────────────────────────────────────────────────────

  async signUp(payload) {
    return this.request("/api/user/create", {
      method: "POST",
      body: JSON.stringify({
        username: payload.email,
        fullName: payload.fullName,
        email: payload.email,
        password: payload.password,
        selectedRole: String(payload.role),
      }),
    });
  }

  async signIn(email, password) {
    return this.request("/api/user/login", {
      method: "POST",
      body: JSON.stringify({ email, password }),
    });
  }

  // ── User / Profile ────────────────────────────────────────────────────────

  async getMe() {
    return this.request("/api/user/me", { method: "GET" });
  }

  async getUserProfile(id) {
    return this.request(`/api/user/${id}`, { method: "GET" });
  }

  async updateUserProfile(id, payload) {
    return this.request(`/api/user/${id}`, {
      method: "PUT",
      body: JSON.stringify({
        fullName: payload.fullName ?? null,
        userName: payload.userName ?? null,
        email: payload.email ?? null,
      }),
    });
  }

  async deleteUserProfile(id) {
    return this.request(`/api/user/${id}`, { method: "DELETE" });
  }

  // ── Students / SLP Clients ────────────────────────────────────────────────

  async addStudent(payload) {
    return this.request("/api/user/student", {
      method: "POST",
      body: JSON.stringify({
        fullName: payload.fullName,
        email: payload.email,
      }),
    });
  }

  async getStudents() {
    return this.request("/api/user/clients", { method: "GET" });
  }

  async getStudent(id) {
    const clients = await this.getStudents();
    const match = clients.find(
      (c) => c.studentId === id || c.studentId === String(id)
    );
    return match || null;
  }

  async getClient(id) {
    return this.request(`/api/user/clients/${id}`, { method: "GET" });
  }

  async deleteSLPClient(slpClientId) {
    return this.request(`/api/user/clients/${slpClientId}`, { method: "DELETE" });
  }

  // ── Assignments ───────────────────────────────────────────────────────────

  async createAssignment(payload) {
    return this.request("/api/user/assignments", {
      method: "POST",
      body: JSON.stringify(payload),
    });
  }

  async getMyAssignments() {
    return this.request("/api/user/assignments/my", { method: "GET" });
  }

  async getStudentAssignments(studentId) {
    return this.request(`/api/user/assignments/student/${studentId}`, { method: "GET" });
  }

  async completeAssignment(assignmentId) {
    return this.request(`/api/user/assignments/${assignmentId}/complete`, { method: "PUT" });
  }

  // ── Sessions ──────────────────────────────────────────────────────────────

  async getSessions() {
    const res = await this.request("/api/session", { method: "GET" });
    return res || [];
  }

  async getSession(sessionId) {
    return this.request(`/api/session/${sessionId}`, { method: "GET" });
  }

  async startSession({ robotId, anonymous = false, userId = null } = {}) {
    return this.request("/api/session", {
      method: "POST",
      body: JSON.stringify({ robotId, anonymous, userId }),
    });
  }

  async getSessionIdFromRobotCode(code) {
    return this.request(`/api/session/code?code=${encodeURIComponent(code)}`, { method: "GET" });
  }

  // Unlike getSessionIdFromRobotCode (GET /code -- anonymous, explicitly
  // documented as not modifying the session), this hits the authenticated
  // GET /join/{code} endpoint that actually claims the session for the
  // current user (sets UserId) -- required so the session's pairing state
  // (session.UserId) reflects reality for anything polling it, e.g. the
  // robot's own pairing-code display.
  async joinSessionByCode(code) {
    return this.request(`/api/session/join/${encodeURIComponent(code)}`, { method: "GET" });
  }

  async endSession(sessionId) {
    return this.request(`/api/session/${sessionId}/end`, { method: "POST" });
  }

  async unpairRobot(sessionId) {
    return this.request(`/api/session/${sessionId}/user`, { method: "DELETE" });
  }

  async getSessionHistory(sessionId) {
    return this.request(`/api/session/${sessionId}/history`);
  }

  async getTrackerEvents(sessionId) {
    return this.request(`/api/session/${sessionId}/tracker-events`, { method: "GET" });
  }

  // ── Session — Robot Membership ────────────────────────────────────────────

  async addRobotToSession(sessionId, robotId) {
    return this.request(`/api/session/${sessionId}/robots`, {
      method: "POST",
      body: JSON.stringify({ robotId }),
    });
  }

  async removeRobotFromSession(sessionId, robotId) {
    return this.request(`/api/session/${sessionId}/robots/${robotId}`, { method: "DELETE" });
  }

  async getSessionRobots(sessionId) {
    return this.request(`/api/session/${sessionId}/robots`, { method: "GET" });
  }

  // ── Session — Robot State ─────────────────────────────────────────────────

  async updateRobotState(sessionId, robotId, state) {
    return this.request(`/api/session/${sessionId}/robots/${robotId}/state`, {
      method: "PUT",
      body: JSON.stringify({
        status: state.status ?? "",
        currentTask: state.currentTask ?? "",
        currentBehaviorId: state.currentBehaviorId ?? null,
        speechLog: state.speechLog ?? "",
      }),
    });
  }

  async getCurrentStates(sessionId) {
    return this.request(`/api/session/${sessionId}/states`, { method: "GET" });
  }

  // ── Robot Idle Mode ────────────────────────────────────────────────────────

  async setRobotIdleMode(sessionId, mode) {
    return this.request(`/api/robot-idle-mode/${sessionId}`, {
      method: "POST",
      body: JSON.stringify({ mode }),
    });
  }

  async stopRobotIdleMode(sessionId) {
    return this.request(`/api/robot-idle-mode/${sessionId}/stop`, { method: "POST" });
  }

  async getRobotIdleMode(sessionId) {
    return this.request(`/api/robot-idle-mode/${sessionId}`, { method: "GET" });
  }

  // ── Lesson Content ────────────────────────────────────────────────────────

  async getLessons() {
    return this.request("/api/lesson/all", { method: "GET" });
  }

  async getLesson(id) {
    return this.request(`/api/lesson/${id}`, { method: "GET" });
  }

  async createLesson(lessonDto) {
    return this.request("/api/lesson/create", {
      method: "POST",
      body: JSON.stringify(lessonDto),
    });
  }

  async updateLesson(lessonId, lessonDto) {
    return this.request(`/api/lesson/${lessonId}`, {
      method: "PUT",
      body: JSON.stringify(lessonDto),
    });
  }

  async generateLessonWithAi(prompt, existingLesson = null) {
    return this.request("/api/lesson/ai/generate", {
      method: "POST",
      body: JSON.stringify({ prompt, existingLesson }),
    });
  }

  // ── Lesson Progress ───────────────────────────────────────────────────────

  async getMyLessonProgress() {
    return this.request("/api/lesson/progress/my", { method: "GET" });
  }

  async getStudentLessonProgress(studentId) {
    return this.request(`/api/lesson/progress/student/${studentId}`, { method: "GET" });
  }

  // ── Lesson Runtime — SLP Commands ─────────────────────────────────────────

  async startLessonSession(sessionId, lessonId, studentId) {
    return this.request(`/api/lesson-runtime/${sessionId}/start`, {
      method: "POST",
      body: JSON.stringify({ lessonId, studentId }),
    });
  }

  async stopLesson(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/stop`, { method: "POST" });
  }

  async skipStep(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/skip`, { method: "POST" });
  }

  async replayStep(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/replay`, { method: "POST" });
  }

  async setStep(sessionId, targetStep) {
    return this.request(`/api/lesson-runtime/${sessionId}/set-step`, {
      method: "POST",
      body: JSON.stringify({ targetStep }),
    });
  }

  async pauseLesson(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/pause`, { method: "POST" });
  }

  async resumeLesson(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/resume`, { method: "POST" });
  }

  async recordSLPFeedback(sessionId, stepId, feedbackCommand) {
    return this.request(`/api/lesson-runtime/${sessionId}/feedback`, {
      method: "POST",
      body: JSON.stringify({ stepId, feedbackCommand }),
    });
  }

  // ── Lesson Runtime — Robot Polling ────────────────────────────────────────

  async getPendingLesson(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/pending-lesson`, { method: "GET" });
  }

  async getPendingStepControl(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/step-control`, { method: "GET" });
  }

  async getPendingFeedback(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/pending-feedback`, { method: "GET" });
  }

  // ── Lesson Runtime — Robot Reporting ──────────────────────────────────────

  async updateLessonProgress(sessionId, dto) {
    return this.request(`/api/lesson-runtime/${sessionId}/progress`, {
      method: "PUT",
      body: JSON.stringify(dto),
    });
  }

  async addNoteToSession(sessionId, note, stepId = 0) {
    return this.request(`/api/lesson-runtime/${sessionId}/interactions`, {
      method: "POST",
      body: JSON.stringify({
        stepId,
        interactionType: "Note",
        studentResponse: note,
        isCorrect: null,
        responseTimeMs: 0,
      }),
    });
  }

  async getLessonInteractions(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/interactions`, { method: "GET" });
  }

  async acknowledgeStepControl(sessionId) {
    return this.request(`/api/lesson-runtime/${sessionId}/step-control`, { method: "DELETE" });
  }

  async acknowledgeFeedback(sessionId, feedbackId) {
    return this.request(`/api/lesson-runtime/${sessionId}/feedback/${feedbackId}/acknowledge`, {
      method: "PUT",
    });
  }

  async getStudentLessonHistory(studentId) {
    const res = await this.request(`/api/lesson-runtime/student/${studentId}/history`, {
      method: "GET",
    });
    return res || [];
  }

  // ── Robots ────────────────────────────────────────────────────────────────

  async registerRobot(payload) {
    return this.request("/api/robot/register", {
      method: "POST",
      body: JSON.stringify({
        name: payload.name,
        model: payload.model,
        serialNumber: payload.serialNumber,
        manufactureDate: payload.manufactureDate,
        firmwareVersion: payload.firmwareVersion,
        ipAddress: payload.ipAddress,
        registeredUserId: payload.registeredUserId ?? null,
      }),
    });
  }

  async updateRobot(id, payload) {
    return this.request(`/api/robot/${id}`, {
      method: "PUT",
      body: JSON.stringify({
        name: payload.name,
        model: payload.model,
        serialNumber: payload.serialNumber,
        manufactureDate: payload.manufactureDate,
        firmwareVersion: payload.firmwareVersion,
        ipAddress: payload.ipAddress,
        registeredUserId: payload.registeredUserId ?? null,
      }),
    });
  }

  async deleteRobot(id) {
    return this.request(`/api/robot/${id}`, { method: "DELETE" });
  }

  async getRobot(id) {
    return this.request(`/api/robot/${id}`, { method: "GET" });
  }

  async getAllRobots() {
    return this.request("/api/robot", { method: "GET" });
  }

  async getRobotsByUserId(userId) {
    return this.request(`/api/robot/user/${userId}`, { method: "GET" });
  }

  async getRobotsByFirmwareVersion(firmwareVersion) {
    return this.request(`/api/robot/firmware/${encodeURIComponent(firmwareVersion)}`, {
      method: "GET",
    });
  }

  async getAvailableBehaviors() {
    return this.request("/api/robot/behaviors", { method: "GET" });
  }

  async getAvailableMotorSequences() {
    return this.request("/api/robot/motorsequences", { method: "GET" });
  }

  // ── Robot Profile (student customization) ───────────────────────────────

  async getMyRobotProfile() {
    return this.request("/api/robotprofile/me", { method: "GET" });
  }

  async updateMyRobotProfile(payload) {
    return this.request("/api/robotprofile/me", {
      method: "PUT",
      body: JSON.stringify({
        nickname: payload.nickname,
        personalityTrait: payload.personalityTrait,
        catchphrase: payload.catchphrase ?? null,
        colorTheme: payload.colorTheme ?? null,
        voice: payload.voice ?? null,
      }),
    });
  }

  async getRobotPersonalityPresets() {
    return this.request("/api/robotprofile/presets", { method: "GET" });
  }

  async getRobotVoicePresets() {
    return this.request("/api/robotprofile/voice-presets", { method: "GET" });
  }

  // ── Robot Voice — Live Session Override ───────────────────────────────────

  async getRobotVoice(sessionId) {
    return this.request(`/api/robot-voice/${sessionId}`, { method: "GET" });
  }

  async setRobotVoice(sessionId, voice) {
    return this.request(`/api/robot-voice/${sessionId}`, {
      method: "POST",
      body: JSON.stringify({ voice }),
    });
  }

  async resetRobotVoice(sessionId) {
    return this.request(`/api/robot-voice/${sessionId}/reset`, { method: "POST" });
  }

  // ── Lesson Content — Visual Aid Upload ───────────────────────────────────

  async uploadVisualAid(file) {
    const formData = new FormData();
    formData.append("file", file);

    const url = `${this.baseUrl}/api/lesson/steps/visual-aid`;
    const response = await fetch(url, {
      method: "POST",
      credentials: "include",
      body: formData,
    });
    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`HTTP ${response.status}: ${errorText}`);
    }
    return response.json();
  }

  // ── RSR Assessment ────────────────────────────────────────────────────────

  async analyzeRsr(formData) {
    const url = `${this.baseUrl}/api/rsr/analyze`;
    const response = await fetch(url, {
      method: "POST",
      credentials: "include",
      body: formData,
    });
    if (!response.ok) {
      const errorText = await response.text();
      throw new Error(`HTTP ${response.status}: ${errorText}`);
    }
    return response.json();
  }

  async getRsrAssessments(studentId) {
    const qs = studentId ? `?studentId=${encodeURIComponent(studentId)}` : "";
    return this.request(`/api/rsr/assessments${qs}`, { method: "GET" });
  }

  async getRsrAssessment(id) {
    return this.request(`/api/rsr/assessments/${id}`, { method: "GET" });
  }

  // ── RSR Speech (sentence audio + robot face playback) ───────────────────────

  async getRsrSentenceManifest() {
    return this.request("/api/rsr-speech/sentences", { method: "GET" });
  }

  async getRobotIdFromCode(code) {
    const { sessionId } = await this.getSessionIdFromRobotCode(code);
    const { robotIds } = await this.getSessionRobots(sessionId);
    if (!robotIds || robotIds.length === 0) {
      throw new Error("No robot is paired with that code.");
    }
    return robotIds[0];
  }

  async queueRsrSentence(robotId, sentenceId) {
    return this.request(`/api/rsr-speech/${robotId}/queue`, {
      method: "POST",
      body: JSON.stringify({ sentenceId }),
    });
  }
}
