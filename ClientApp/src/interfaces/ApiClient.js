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


async signUp(payload) {
    const body = {
      username: payload.email,
      fullName: payload.fullName,
      email: payload.email,
      password: payload.password,
      selectedRole: String(payload.role),
    };

    const res = await this.request("/api/account/create", {
      method: "POST",
      body: JSON.stringify(body),
    });

    return res;
}

  async addStudent(payload) {
    const body = {
      fullName: payload.fullName,
      email: payload.email,
    };

    const res = await this.request("/api/account/create/student", {
      method: "POST",
      body: JSON.stringify(body),
    });
    return res;
  }

async signIn(email, password) {
    const payload = { email, password };

    const data = await this.request("/api/account/login", {
      method: "POST",
      body: JSON.stringify(payload),
    });

      return data;
    }

      async getSessions() {
        const res = await this.request(`/api/robotsession`, { method: "GET" });
        return res || [];
    }

      async getSessionHistory(sessionId) {
          const res = await this.request(`/api/robotsession/${sessionId}/history`);
          return res;
      }


  async getUserProfile(id) {
    const res = await this.request(`/api/account/${id}`, {
      method: "GET"
    });
    return res;
  }

  // Get all students (for teacher dashboard)
  async getStudents() {
    return this.request("/api/SLPClient", {
      method: "GET",
    });
  }

  // get student by ID
  async getStudent(id) {
    const clients = await this.getStudents();

    const match = clients.find(c => 
      c.studentId === id || c.studentId === String(id)
    );

    return match || null;
  }

 async getLessons() {
  const res = await this.request(`/api/lesson/all`, {
    method: "GET",
  });
  return res;
}

  //temp
  async getLesson(id) {
    const res = await this.request(`/api/lesson/${id}`, {
      method: "GET",
    });
    return res;
  }

  //temp
  async addNoteToSession(sessionId, note, stepId = 0) {
    const res = await this.request(`/api/lessoninteraction/${sessionId}`, {
      method: "POST",
      body: JSON.stringify({
        stepId,
        interactionType: "Note",
        studentResponse: note,
        isCorrect: null,
        responseTimeMs: 0,
      }),
    });
    return res;
  }

  //temp
  async updateStudentProgress(studentId, progressData) {
    const res = await this.request(`/api/student/${studentId}/progress`, {
      method: "POST",
      body: JSON.stringify(progressData),
    });
    return res;
  }

  async startSession({ anonymous = false } = {}) {
    const res = await this.request("/api/robotsession", {
      method: "POST",
      body: JSON.stringify({ anonymous }),
    });
    return res;
  }

  async getSessionIdFromRobotCode(robotCode) {
    const res = await this.request(
      `/api/robotsession/join/${encodeURIComponent(robotCode)}`,
      {
        method: "GET",
      }
    );
    return res;
  }

  async addRobotToSession(sessionId, robotId, currentState = {}) {
    const body = {
      robotId,
      currentState: {
        status: currentState.status ?? "",
        currentTask: currentState.currentTask ?? "",
        currentBehaviorId: currentState.currentBehaviorId ?? null,
        speechLog: currentState.speechLog ?? ""
      }
    };

    const res = await this.request(`/api/robotsession/${sessionId}/robots`, {
      method: "POST",
      body: JSON.stringify(body),
    });
    return res;
  }

  async updateRobotState(sessionId, robotId, state) {
    const body = {
      status: state.status ?? "",
      currentTask: state.currentTask ?? "",
      currentBehaviorId: state.currentBehaviorId ?? null,
      speechLog: state.speechLog ?? ""
    };

    const res = await this.request(`/api/robotsession/${sessionId}/robots/${robotId}/state`, {
      method: "PUT",
      body: JSON.stringify(body),
    });
    return res;
  }

  async getCurrentStates(sessionId) {
    const res = await this.request(`/api/robotsession/${sessionId}/states`, {
      method: "GET",
    });
    return res;
  }

  async removeRobotFromSession(sessionId, robotId) {
    const res = await this.request(`/api/robotsession/${sessionId}/robots/${robotId}`, {
      method: "DELETE",
    });
    return res;
  }

  async endSession(sessionId) {
    const res = await this.request(`/api/robotsession/${sessionId}/end`, {
      method: "POST",
    });
    return res;
  }


  async registerRobot(payload) {
  const body = {
    name: payload.name,
    model: payload.model,
    serialNumber: payload.serialNumber,
    manufactureDate: payload.manufactureDate, // ISO string recommended
    firmwareVersion: payload.firmwareVersion,
    ipAddress: payload.ipAddress,
    registeredUserId: payload.registeredUserId ?? null,
  };

  const res = await this.request("/api/robot/register", {
    method: "POST",
    body: JSON.stringify(body),
  });
  return res;
}

  async updateRobot(id, payload) {
    const body = {
      name: payload.name,
      model: payload.model,
      serialNumber: payload.serialNumber,
      manufactureDate: payload.manufactureDate,
      firmwareVersion: payload.firmwareVersion,
      ipAddress: payload.ipAddress,
      registeredUserId: payload.registeredUserId ?? null,
    };

    const res = await this.request(`/api/robot/${id}`, {
      method: "PUT",
      body: JSON.stringify(body),
    });
    return res;
  }

  async deleteRobot(id) {
    const res = await this.request(`/api/robot/${id}`, { method: "DELETE" });
    return res;
  }

  async getRobot(id) {
    const res = await this.request(`/api/robot/${id}`, { method: "GET" });
    return res;
  }

  async getAllRobots() {
    const res = await this.request(`/api/robot`, { method: "GET" });
    return res;
  }

  async getRobotsByUserId(userId) {
    const res = await this.request(`/api/robot/user/${userId}`, { method: "GET" });
    return res;
  }

  async getRobotsByFirmwareVersion(firmwareVersion) {
    const res = await this.request(`/api/robots/firmware/${encodeURIComponent(firmwareVersion)}`, {
      method: "GET",
    });
    return res;
  }

  async createLesson(payload) {
    const body = {
      title: payload.title,
      description: payload.description ?? "",
      lessonType: payload.lessonType,
      lessonDescription: payload.lessonDescription,
      createdById: payload.createdById,
    };

    const res = await this.request("/api/lesson/create", {
      method: "POST",
      body: JSON.stringify(body),
    });
    return res;
  }

  async startLessonSession(lessonId, sessionId) {
    const res = await this.request(`/api/LessonSession/${sessionId}/lesson`, {
      method: "POST",
      body: JSON.stringify({ lessonId }),
    });
    return res;
  }

  async createLesson(lessonDto) {
    return this.request("/api/lesson/create", {
      method: "POST",
      body: JSON.stringify(lessonDto),
    });
  }

  async getTrackerEvents(sessionId) {
    const res = await fetch(`${this.baseUrl}/api/RobotSession/${sessionId}/tracker-events`, {
      credentials: "include"
    });

    if (!res.ok) {
      throw new Error("Failed to fetch tracker events");
    }

    return await res.json();
  }

  async getLessonInteractions(sessionId) {
    return this.request(`/api/LessonInteraction/${sessionId}`, {
      method: "GET",
    });
  }

  async getStudentLessonHistory(studentId) {
    const res = await this.request(`/api/LessonSession/student/${studentId}/history`, {
      method: "GET",
    });

    return res || [];
  }

  async deleteSLPClient(slpClientId) {
    return this.request(`/api/SLPClient/${slpClientId}`, {
      method: "DELETE",
    });
  }
}