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

    // const response = await fetch(url, { ...options, headers });

    const response = await fetch(url, {
      credentials: "include",
      ...options,
      headers
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
      userName: payload.email,
      fullName: payload.fullName,
      email: payload.email,
      password: payload.password,
      selectedRole: String(payload.role),
    };

    const res = await this.request("/api/accounts/create", {
      method: "POST",
      body: JSON.stringify(body),
    });

    return res;
  }

  //temp
  async addStudent(payload) {
    const body = {
      userName: payload.email,
      fullName: payload.fullName,
      email: payload.email,
      password: payload.password,
      selectedRole: "STUDENT",
    };

      const res = await this.request("/api/accounts/create", {
        method: "POST",
        body: JSON.stringify(body),
      });
      return res;
  }

  async signIn(email, password) {
      const payload = { email, password };

      const data = await this.request("/api/accounts/login", {
        method: "POST",
        body: JSON.stringify(payload),
      });

      return data;
    }

      async getSessions() {
        const res = await this.request(`/api/robotsessions`, { method: "GET" });
        return res || [];
      }

      async getSessionHistory(sessionId) {
          const res = await this.request(`/api/robotsessions/${sessionId}/history`);
          return res;
      }


  async getUserProfile(id) {
      const res = await this.request(`/api/accounts/${id}`, {
        method: "GET",
      });
      return res;
  }

  //temp
  async getStudents(id) {
    const res = await this.request(`/api/accounts/${id}`, {
      method: "GET",
    });
    return res;
  }

  //temp
  async getStudent(id) {
    const res = await this.request(`/api/students/${id}`, {
      method: "GET",
    });
    return res;
  }

  //temp
  async getLessons() {
    const res = await this.request(`/api/lessons`, {
      method: "GET",
    });
    return res;
  }

  //temp
  async getLesson(id) {
    const res = await this.request(`/api/lessons/${id}`, {
      method: "GET",
    });
    return res;
  }

  //temp
  async addNoteToSession(sessionId, note) {
    const res = await this.request(`/api/robotsessions/${sessionId}/notes`, {
      method: "POST",
      body: JSON.stringify({ note }),
    });
    return res;
  }

  //temp
  async updateStudentProgress(studentId, progressData) {
    const res = await this.request(`/api/students/${studentId}/progress`, {
      method: "POST",
      body: JSON.stringify(progressData),
    });
    return res;
  }

  async startSession({ anonymous = false } = {}) {
    const res = await this.request("/api/robotsessions", {
      method: "POST",
      body: JSON.stringify({ anonymous }),
    });
    return res;
  }

  async getSessionIdFromRobotCode(robotCode) {
    const res = await this.request(`/api/robotsessions/join/${encodeURIComponent(robotCode)}`, {
      method: "GET",
    });
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

    const res = await this.request(`/api/robotsessions/${sessionId}/robots`, {
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

    const res = await this.request(`/api/robotsessions/${sessionId}/robots/${robotId}/state`, {
      method: "PUT",
      body: JSON.stringify(body),
    });
    return res;
  }

  async getCurrentStates(sessionId) {
    const res = await this.request(`/api/robotsessions/${sessionId}/states`, {
      method: "GET",
    });
    return res;
  }

  async removeRobotFromSession(sessionId, robotId) {
    const res = await this.request(`/api/robotsessions/${sessionId}/robots/${robotId}`, {
      method: "DELETE",
    });
    return res;
  }

  async endSession(sessionId) {
    const res = await this.request(`/api/robotsessions/${sessionId}/end`, {
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

  const res = await this.request("/api/robots/register", {
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

    const res = await this.request(`/api/robots/${id}`, {
      method: "PUT",
      body: JSON.stringify(body),
    });
    return res;
  }

  async deleteRobot(id) {
    const res = await this.request(`/api/robots/${id}`, { method: "DELETE" });
    return res;
  }

  async getRobot(id) {
    const res = await this.request(`/api/robots/${id}`, { method: "GET" });
    return res;
  }

  async getAllRobots() {
    const res = await this.request(`/api/robots`, { method: "GET" });
    return res;
  }

  async getRobotsByUserId(userId) {
    const res = await this.request(`/api/robots/user/${userId}`, { method: "GET" });
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

    const res = await this.request("/api/lessons/create", {
      method: "POST",
      body: JSON.stringify(body),
    });
    return res;
  }

}