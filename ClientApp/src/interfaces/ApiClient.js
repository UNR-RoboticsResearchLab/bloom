// ApiClient.js
// interface used to communicate with api, provided by ApiClientContext, with useApiClient()

export default class ApiClient
{
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
      
        // try to parse JSON, but allow empty responses
        try {
          return await response.json();
        } catch {
          return null;
        }
    }

    async signUp(username, fullName, email, password, role, navigate, setErr) {

        console.log("APICLIENT");
        var user = {
            username: username,
            fullName: fullName,
            email: email,
            password: password,
            selectedRole: role
        }

        try {
            const response = await fetch(`${this.baseurl}/account/create`, {
            method: "POST",
            headers: { "Content-Type": "application/json" },
            body: JSON.stringify(user),
            });

            if (response.ok) {
                // navigate("/login", { replace: true });
            } else {
                const data = await response.json();
                setErr(data.message || "Sign up failed");
            }
        } catch (err) {
            setErr("Network error");
            console.error(err);
        }

        
    }

    async signIn(email, password) {
        const res = await fetch(`${this.baseurl}/account/login`, {
            method: 'POST',
            headers: { 'Content-Type': 'application/json' },
            body: JSON.stringify({ email, password }),
        }).then(
            (response) => {
                if (response.ok) {
                    // navigate('/dashboard', { replace: true });
                }
                else {
                    return response.json().then((data) => {
                        // setErr(data.message || "Sign in failed");
                    });
                }
            }
        );

        return res;
    }

    async getUserProfile(id) {
        const res = await this.request(`/account/${id}`);
        return res;
    }

}
