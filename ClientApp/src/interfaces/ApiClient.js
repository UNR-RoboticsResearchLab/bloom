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

    const response = await fetch(url, { ...options, headers });

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

    const res = await this.request("/api/accounts/create", {
      method: "POST",
      body: JSON.stringify(body),
    });

    return res;
}

//temp
async addStudent(payload) {
    const body = {
      name: payload.name,
      level: payload.level,
      status: payload.status,
    };

    const res = await this.request("/api/students/create", {
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
        return res || [];
        const res = await this.request(`/api/robotsessions`);
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

  async getStudents() {
    const res = await this.request(`/api/students`, {
      method: "GET",
    });
    return res;
  }
}